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
public class AprilTagFollowingTurret extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    private DcMotor backLeft, backRight, frontLeft, frontRight;
    private DcMotor motor;  // Turret motor

    // === PID Constants for Turret Control ===
    private static final double Kp = 0.02;    // Proportional gain (response speed)
    private static final double Ki = 0.0;     // Integral gain (not used here)
    private static final double Kd = 0.06;   // Derivative gain (dampens overshoot)

    // === PID State Variables ===
    private double integral = 0;
    private double lastError = 0;

    // Minimum power to overcome turret motor friction
    private static final double MIN_POWER = 0.07;

    // How close to center the tag must be to stop correcting (in degrees)
    private static final double CENTER_THRESHOLD = 5.0;

    @Override
    public void init() {
        // Initialize Limelight and IMU
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(IMU.class, "imu");

        // Optional IMU configuration (not used here but good to have)
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        // Drive motors
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        // Turret motor
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setDirection(DcMotor.Direction.REVERSE); // Change to FORWARD if needed

        // Set motor directions for mecanum drive
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Activate Limelight pipeline
        limelight.pipelineSwitch(0);
    }

    @Override
    public void start() {
        // Start Limelight streaming
        limelight.start();
    }

    @Override
    public void loop() {
        // === APRILTAG TURRET TRACKING (PID) ===
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            double tx = llResult.getTx(); // Horizontal offset in degrees (-27 to +27)
            boolean isCentered = Math.abs(tx) < CENTER_THRESHOLD;

            double turretPower = 0;

            if (!isCentered) {
                // PID control logic
                double error = tx;
                integral += error; // Integrate error (not used if Ki = 0)
                double derivative = error - lastError;

                // PID formula to calculate output
                turretPower = -(Kp * error + Ki * integral + Kd * derivative);

                // Apply minimum power if below threshold to overcome motor friction
                if (Math.abs(turretPower) < MIN_POWER) {
                    turretPower = MIN_POWER * Math.signum(turretPower);
                }

                // Clamp output to valid range
                turretPower = Math.max(-1.0, Math.min(1.0, turretPower));

                lastError = error; // Store error for next loop
            }

            // Apply power to turret motor
            motor.setPower(turretPower);

            // Debug/telemetry info
            Pose3D botPose = llResult.getBotpose_MT2();
            telemetry.addData("Tag Detected", "YES");
            telemetry.addData("tx (degrees)", tx);
            telemetry.addData("Is Centered", isCentered ? "YES" : "NO");
        } else {
            // No tag found — stop turret
            motor.setPower(0);
            telemetry.addData("Tag Detected", "NO");
        }

        // === MECANUM DRIVE CONTROL ===
        double y = -gamepad1.left_stick_y;   // Forward/backward
        double x = gamepad1.left_stick_x;    // Strafe left/right
        double turn = gamepad1.right_stick_x; // Rotate in place

        // Calculate motor powers for mecanum drive
        double frontLeftPower = y + x + turn;
        double frontRightPower = y - x - turn;
        double backLeftPower = y - x + turn;
        double backRightPower = y + x - turn;

        // Normalize motor powers to stay within [-1.0, 1.0]
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

        // Set powers to motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.update(); // Send telemetry to driver station
    }
}
