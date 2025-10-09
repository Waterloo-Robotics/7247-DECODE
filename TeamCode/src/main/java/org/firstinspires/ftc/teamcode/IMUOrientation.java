

/* THIS IS NOT A TELEOP, THIS IS IMU CODE! PLEASE DO NOT MODIFY UNLESS YOU KNOW WHAT YOU ARE DOING, BECAUSE THIS TOOK ME FOREVER, AND I DO NOT WANT IT TO MESS UP

IF WORK = NO TOUCH
TOUCH = BREAK
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IMUOrientation {

    private IMU imu;


    public void init(HardwareMap hwMap) {
        imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT

        );

        imu.initialize(new IMU.Parameters(RevOrientation));

    }

    public double getHeading(AngleUnit degrees) {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

    }


    public class PinpointTest extends LinearOpMode {

        GoBildaPinpointDriver pinpoint;

        @Override
        public void runOpMode() {
            // Create the pinpoint driver, assuming it’s on "i2c pin 0" (configure in RC app!)
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

            // Reset position tracking before start
            pinpoint.resetPosAndIMU();
            telemetry.addData("IMU",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            waitForStart();

            Pose2D pose = pinpoint.getPosition();  // returns a Pose2D object

            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);
            while (opModeIsActive()) {
                // Update the pinpoint sensor
                pinpoint.update();


                // Heading in radians or degrees depending on your setup
                double headingDeg = Math.toDegrees(pinpoint.getHeading(AngleUnit.DEGREES));

                telemetry.addData("X (mm)", x);
                telemetry.addData("Y (mm)", y);
                telemetry.addData("Heading (deg)", headingDeg);
                telemetry.update();
            }
        }
    }





}