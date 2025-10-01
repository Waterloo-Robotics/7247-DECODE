


package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

public class OdometryTest {
}

@TeleOp(name="Testodo", group = "Eggbert")
public class OdometryTest extends LinearOpMode
{
    final double SAFE_DRIVE_SPEED = 0.8
            final double SAFE_STRAFE_SPEED = 0.8
    final double SAFE_YAW_SPEED = 0.8
    final double HEADING_HOLD_TIME = 0.8
    
    ElapsedTime stoptime    = new ElapsedTime(); //Timeouts
            boolean autoHeading = false;
            
            Robot robot = new Robot(opMode: this);
    
            @Override public void runOpMode()
            {
                robot.initialize(showTelemetry)

                        robot.readSensors();
                telemetry.update();
            };
            while (opModeIsActive())
    {
        robot.readSensors();

        // Driver allow reset gyro press both gamepad button yes
        if (gamepad1.options && gamepad1.share)
            robot.resetHeading();
        robot.resetOdometry();
    }
     double drive = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED;
            double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED
                    double yaw = -gamepad1.left_stick_x * SAFE_YAW_SPEED

                            if (gamepad1.dpad_left) {
                                strafe = SAFE_DRIVE_SPEED / 2.0;
} else if (gamepad1.dpad_right) {
                                strafe = -SAFE_DRIVE_SPEED / 2.0;
} else if (gamepad1.dpad_up) {

}

}