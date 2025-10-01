package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* THIS TELEOP IS FOR TESTING IF THE IMU IS WORKING CORRECTLY. */

@TeleOp
public class IMUTeleop extends OpMode{

    TestIMU bench = new TestIMU();

    // needed
    private boolean tag_found_21;
    private boolean tag_found_22;
    private boolean tag_found_23;

    //
    @Override
    public void init()
    {
        bench.init(hardwareMap);

        // needed at start
        tag_found_21 = false;
        tag_found_22= false;
        tag_found_23 = false;
    }

// detected pattern and whether pattern was detected or not
    String detectedPattern = "None";
    boolean locked = false;

    // loop while opmode is active, reads off detected pattern and keeps it that way
    @Override
    public void loop() {
        telemetry.addData("Heading", bench.getHeading(AngleUnit.DEGREES));
        if (tag_found_21)
        {
            detectedPattern = "GREEN PURPLE PURPLE";
            locked = true;
        }
        else if (tag_found_22)
        {
            detectedPattern = "PURPLE GREEN PURPLE";
            locked = true;
        }
        else if (tag_found_23){
            detectedPattern = "PURPLE PURPLE GREEN";
            locked = true;
        }

        telemetry.addData("Pattern", detectedPattern);
        telemetry.update();

    }
}



