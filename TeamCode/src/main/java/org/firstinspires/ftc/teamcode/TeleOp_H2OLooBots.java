package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.modules.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.modules.flywheelModule;

@TeleOp(name="H2O Loo Bots Teleop", group="LinearOpMode")
public class TeleOp_H2OLooBots extends OpMode {

    /* ---------- Drive Motors ---------- */
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor flywheel;
    private DcMotor intake;
    private DcMotor transfer;
    private Servo hood;

    /* ---------- Modules & Sensors ---------- */
    private flywheelModule flywheelControl;
    private Limelight3A limelight;
    private LimelightProcessingModule llModule;
    private Servo leftLED;
    private Servo rightLED;

    /* ---------- Variables ---------- */
    private double hoodPosition = 0.4; // start in mid position
    private double flywheelRPM;

    // Intake state management
    private enum IntakeState { OFF, INTAKE, REVERSE }
    private IntakeState intakeState = IntakeState.OFF;

    // Transfer control
    private boolean transferForward = false;   // forward toggle state
    private boolean forwardPressedLast = false; // for edge detection

    @Override
    public void init() {
        /* ----- Hardware Map ----- */
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        hood = hardwareMap.get(Servo.class, "hood");
        leftLED = hardwareMap.get(Servo.class, "leftLED");
        rightLED = hardwareMap.get(Servo.class, "rightLED");

        // Mecanum motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        // Modules
        flywheelControl = new flywheelModule(flywheel);
        flywheelRPM = 0;

        llModule = new LimelightProcessingModule(limelight, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void loop() {
        /* ---------------- DRIVE CODE ---------------- */
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        // keep LEDs purple
        leftLED.setPosition(0.722);
        rightLED.setPosition(0.722);

        double frontLeftPower = y + x + turn;
        double frontRightPower = y - x - turn;
        double backLeftPower = y - x + turn;
        double backRightPower = y + x - turn;

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
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

        /* ---------------- FLYWHEEL CONTROL ---------------- */
        flywheelRPM += gamepad2.right_trigger * 50;
        flywheelRPM -= gamepad2.left_trigger * 50;

        if (gamepad2.y) flywheelRPM = 0;

        flywheelRPM = Math.max(0, Math.min(4200, flywheelRPM));
        flywheelControl.set_speed((int) flywheelRPM);

        /* ---------------- INTAKE CONTROL ---------------- */
        double intakePower = 0.0;

        // Toggle intake on/off with right bumper
        if (gamepad2.right_bumper) {
            if (intakeState == IntakeState.INTAKE) {
                intakeState = IntakeState.OFF;
            } else {
                intakeState = IntakeState.INTAKE;
            }
        }

        // Reverse intake while left bumper is held
        if (gamepad2.left_bumper) {
            intakeState = IntakeState.REVERSE;
        } else if (intakeState == IntakeState.REVERSE && !gamepad2.left_bumper) {
            intakeState = IntakeState.INTAKE;
        }

        // Determine motor powers based on intake state
        switch (intakeState) {
            case OFF:
                intakePower = 0.0;
                break;
            case INTAKE:
                intakePower = 1.0;
                break;
            case REVERSE:
                intakePower = -1.0;
                break;
        }

        intake.setPower(intakePower);

        /* ---------------- TRANSFER CONTROL ---------------- */
        double transferPower = 0.0;

        boolean forwardPressed = gamepad2.b; // toggle forward
        boolean reverseHeld = gamepad2.x;    // hold to reverse

        // Toggle transfer forward mode on rising edge of B
        if (!forwardPressedLast && forwardPressed) {
            transferForward = !transferForward;
        }

        // Reverse overrides forward toggle while held
        if (reverseHeld) {
            transferPower = -1.0;
        } else if (transferForward) {
            transferPower = 1.0;
        } else {
            transferPower = 0.0;
        }

        // save last state for edge detection
        forwardPressedLast = forwardPressed;

        // Apply motor power
        transfer.setPower(transferPower);

        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Transfer Forward", transferForward);
        telemetry.addData("Transfer Power", transferPower);

        /* ---------------- HOOD CONTROL ---------------- */
        if (gamepad2.dpad_up) {
            hoodPosition += 0.005;
        } else if (gamepad2.dpad_down) {
            hoodPosition -= 0.005;
        }

        hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));

        if (gamepad2.dpad_left) {
            hoodPosition = 0.417;
            flywheelRPM = 3500;
        }

        if (gamepad2.dpad_right) {
            hoodPosition = 0.25;
            flywheelRPM = 2500;
        }

        if (gamepad2.a) flywheelRPM = 0;

        hood.setPosition(hoodPosition);
        flywheelControl.set_speed((int) flywheelRPM);

        /* ---------------- LIMELIGHT TELEMETRY ---------------- */
        Pose2D pose = llModule.limelightResult();

        if (pose != null) {
            telemetry.addData("X (inches)", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (inches)", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Rotation (degrees)", pose.getHeading(AngleUnit.DEGREES));
        } else {
            telemetry.addData("Limelight", "No valid target");
        }

        /* ---------------- GENERAL TELEMETRY ---------------- */
        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("PID Error", flywheelControl.pid_controller.error);
        telemetry.addData("Motor Speed", flywheelControl.motor_speed_rpm);
        telemetry.addData("Feedforward", flywheelControl.feedforward_power);
        telemetry.addData("PID Power", flywheelControl.pid_power);
        telemetry.addData("Hood Pos", hood.getPosition());
        telemetry.update();
    }
}
