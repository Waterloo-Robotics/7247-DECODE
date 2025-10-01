package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class AprilTagFollowingWheels extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private DcMotor backLeft, backRight, frontLeft, frontRight;

    // Proportional control constant for rotation (slower turning)
    private static final double Kp = 0.02;  // Reduced to make the turn slower

    // Flag for tracking if a tag is detected
    private boolean tagDetected = false;

    // Threshold for considering the tag as "roughly centered"
    private static final double CENTER_THRESHOLD = 5.0;  // Tag X value should be within -5 to 5 to be considered centered

    @Override
    public void init() {
        // Initialize hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        // Set up IMU
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        // Set up motors
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        // limelight pipeline - we arent using this limelight for multiple bots so dont change for now
        limelight.pipelineSwitch(0);

        // motor directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        // Get the latest result from Limelight
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // Tag is detected
            tagDetected = true;

            // Get the horizontal displacement (Tx) from Limelight
            double tx = llResult.getTx();

            // Check if the tag is roughly centered
            boolean isRoughlyCentered = Math.abs(tx) < CENTER_THRESHOLD;

            // If it's not roughly centered, rotate to center it
            double turn = 0;
            if (!isRoughlyCentered) {
                // Use proportional control to adjust the robot's turn
                turn = -Kp * tx;  // Negative for right turn, positive for left turn
            }

            // Apply the turn to the mecanum drive motors
            mecanumDrive(0, 0, turn);

            // Show tag info and centering status on telemetry
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tag Detected", "YES");
            telemetry.addData("Tag X", llResult.getTx());
            telemetry.addData("Tag Y", llResult.getTy());
            telemetry.addData("Tag Area", llResult.getTa());
            telemetry.addData("Roughly Centered", isRoughlyCentered ? "YES" : "NO");
        } else {
            // No tag detected, stop the robot
            tagDetected = false;

            // Stop the robot by setting motor powers to zero
            mecanumDrive(0, 0, 0);

            // Display "No Tag Detected" in telemetry
            telemetry.addData("Tag Detected", "NO");
        }

        telemetry.update();
    }

    private void mecanumDrive(double x, double y, double turn) {
        // Mecanum drive logic (adjusts motor powers based on inputs)
        double frontLeftPower = y + x + turn;
        double frontRightPower = y - x - turn;
        double backLeftPower = y - x + turn;
        double backRightPower = y + x - turn;

        //private void mecanumDrive(double x, double y, double turn) {
        //        // Mecanum drive logic (adjusts motor powers based on inputs)
        //        double frontLeftPower = y + x + turn;
        //        double frontRightPower = y - x - turn;
        //        double backLeftPower = y - x + turn;
        //        double backRightPower = y + x - turn;
        //
        //        // Normalize motor powers to be within [-1, 1]
        //        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
        //                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        //        if (maxPower > 1.0) {
        //            frontLeftPower /= maxPower;
        //            frontRightPower /= maxPower;
        //            backLeftPower /= maxPower;
        //            backRightPower /= maxPower;
        //        } Normalize motor powers to be within [-1, 1]
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);


    }
}
