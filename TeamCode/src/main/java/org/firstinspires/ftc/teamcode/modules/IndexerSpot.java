package org.firstinspires.ftc.teamcode.modules;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IndexerSpot {
    /* IndexerSpot
    /* This class represents a single spot in the indexer. */
    private Servo servo;
    private RevColorSensorV3 color_a;
    private RevColorSensorV3 color_b;

    public ArtifactStatus artifact_status = ArtifactStatus.EMPTY;

    private enum ModuleStates {
        HOME,
        SHOOTING,
        RETURNING_HOME;
    }

    private ModuleStates module_state = ModuleStates.HOME;
    private final ElapsedTime servo_timer = new ElapsedTime();

    public IndexerSpot(Servo spot_servo,
                       RevColorSensorV3 spot_color_a,
                       RevColorSensorV3 spot_color_b)
    {
        this.servo = spot_servo;
        this.color_a = spot_color_a;
        this.color_b = spot_color_b;
    }

    public void updateArtifactStatus()
    {
        int red   = (this.color_a.red()   + this.color_b.red())   / 2;
        int green = (this.color_a.green() + this.color_b.green()) / 2;
        int blue  = (this.color_a.blue()  + this.color_b.blue())  / 2;

        // Use distance to detect if a ball is present
        double avgDistance = (this.color_a.getDistance(DistanceUnit.CM) +
                this.color_b.getDistance(DistanceUnit.CM)) / 2.0;

        if (avgDistance > 3.0) {
            this.artifact_status = ArtifactStatus.EMPTY;
        }
        else if (green > red + 50 && green > blue + 50) {
            this.artifact_status = ArtifactStatus.GREEN;
        } else if (red + blue > green + 80) {
            this.artifact_status = ArtifactStatus.PURPLE;
        } else {
            this.artifact_status = ArtifactStatus.UNKNOWN;
        }
    }

    public void shoot()
    {
        this.servo.setPosition(1);
        this.servo_timer.reset();
        this.servo_timer.startTime();

        this.module_state = ModuleStates.SHOOTING;
    }

    public boolean isHome(){
        return (this.module_state == ModuleStates.HOME);
    }

    public void update()
    {
        this.updateArtifactStatus();

        switch (this.module_state)
        {
            case SHOOTING:
                /* If enough time has elapsed for the artifact to put into the
                /* turret, then reset the timer and lower the arm */
                if (this.servo_timer.time() > 1)
                {
                    this.servo_timer.reset();
                    this.servo_timer.startTime();

                    this.servo.setPosition(0);
                    this.module_state = ModuleStates.RETURNING_HOME;
                }
                break;

            case RETURNING_HOME:
                /* If enough time has elapsed for the arm to return home,
                /* stop the timer and return status to home */
                if (this.servo_timer.time() > 0.5)
                {
                    this.servo_timer.reset();

                    this.module_state = ModuleStates.HOME;
                }
                break;
        }

    }
}