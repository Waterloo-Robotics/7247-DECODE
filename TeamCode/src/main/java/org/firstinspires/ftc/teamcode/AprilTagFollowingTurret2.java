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
public class AprilTagFollowingTurret2 extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private DcMotor backLeft, backRight, frontLeft, frontRight;
    private DcMotor motor;  // Turret motor

    // === PID Constants for Turret Control ===
    private static final double Kp = 0.02;
    private static final double Ki = 0.0;
    private static final double Kd = 0.0015;

    // === PID State Variables ===
    private double integral = 0;
    private double lastError = 0;

    // Thresholds
    private static final double MIN_POWER = 0.05;
    private static final double CENTER_THRESHOLD = 1.5; // tighter = stops better

    @Override
    public void init() {
        // Initialize hardware
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientation));

        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        limelight.start();
        integral = 0;
        lastError = 0;
    }

    @Override
    public void loop() {
        // === AprilTag Tracking ===
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx(); // Horizontal offset in degrees
            boolean isCentered = Math.abs(tx) <= CENTER_THRESHOLD;

            double turretPower = 0;

            if (!isCentered) {
                double error = tx;
                integral += error;
                double derivative = error - lastError;

                turretPower = -(Kp * error + Ki * integral + Kd * derivative);

                // Apply minimum power only if needed
                if (Math.abs(turretPower) < MIN_POWER) {
                    turretPower = MIN_POWER * Math.signum(turretPower);
                }

                turretPower = Math.max(-1.0, Math.min(1.0, turretPower));
                lastError = error;
            } else {
                turretPower = 0;
                integral = 0; // reset PID to prevent windup
                lastError = 0;
            }

            motor.setPower(turretPower);

            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tag Detected", "YES");
            telemetry.addData("tx (°)", "%.2f", tx);
            telemetry.addData("Turret Power", "%.2f", turretPower);
            telemetry.addData("Is Centered", isCentered ? "YES" : "NO");

        } else {
            motor.setPower(0);
            telemetry.addData("Tag Detected", "NO");
        }

        // === Mecanum Drive ===
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double frontLeftPower = y + x + turn;
        double frontRightPower = y - x - turn;
        double backLeftPower = y - x + turn;
        double backRightPower = y + x - turn;

        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
