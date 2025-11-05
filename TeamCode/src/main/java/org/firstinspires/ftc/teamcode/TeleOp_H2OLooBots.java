package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private ColorSensor colorSensor;
    private Servo leftLED;
    private Servo rightLED;

    /* ---------- Variables ---------- */
    private double hoodPosition = 0.4; // start in mid position
    private double flywheelRPM;

    // Intake state management
    private enum IntakeState { OFF, INTAKE, REVERSE }
    private IntakeState intakeState = IntakeState.OFF;

    private static final double GREEN_POS = 0.5;
    private static final double BLUE_POS = 0.611;
    private static final double PURPLE_POS = 0.772;
    private long lastSeenTime = 0;

    @Override
    public void init() {
        /* ----- Hardware Map ----- */
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer"); // Initialize transfer motor
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        hood = hardwareMap.get(Servo.class, "hood");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        leftLED = hardwareMap.get(Servo.class, "leftLED");
        rightLED = hardwareMap.get(Servo.class, "rightLED");

        // Mecanum motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.REVERSE);


        // Modules
        flywheelControl = new flywheelModule(flywheel);
        flywheelRPM = 0;

        llModule = new LimelightProcessingModule(limelight, telemetry);

        setLEDs(BLUE_POS);
        lastSeenTime = System.currentTimeMillis();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        /* ---------------- DRIVE CODE ---------------- */
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

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
        flywheelRPM += gamepad1.right_trigger * 50;
        flywheelRPM -= gamepad1.left_trigger * 50;

        if (gamepad1.y) flywheelRPM = 0;

        flywheelRPM = Math.max(0, Math.min(4200, flywheelRPM));
        flywheelControl.set_speed((int) flywheelRPM);

        /* ---------------- INTAKE CONTROL ---------------- */
        if (gamepad1.rightBumperWasPressed()) {
            intakeState = (intakeState == IntakeState.INTAKE) ? IntakeState.OFF : IntakeState.INTAKE;
        }

        if (gamepad1.left_bumper) {
            intakeState = IntakeState.REVERSE;

        } else if (intakeState == IntakeState.REVERSE) {
            intakeState = IntakeState.INTAKE;
        }

        // Apply motor power for intake
        switch (intakeState) {
            case OFF:
                intake.setPower(0.0);
                transfer.setPower(0.0);
                break;
            case INTAKE:
                intake.setPower(1.0);
                transfer.setPower(1.0); // forward with intake
                break;
            case REVERSE:
                intake.setPower(-1.0);
                transfer.setPower(-1.0); // reverse with intake
                break;
        }

        /* ---------------- TRANSFER MOTOR CONTROL ---------------- */


        /* ---------------- HOOD CONTROL ---------------- */
        if (gamepad1.dpad_up) {
            hoodPosition += 0.005;
        } else if (gamepad1.dpad_down) {
            hoodPosition -= 0.005;
        }

        hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));

        if (gamepad1.dpad_left) {
            hoodPosition = 0.417;
            flywheelRPM = 3500;
        }

        if (gamepad1.dpad_right) {
            hoodPosition = 0.25;
            flywheelRPM = 2500;
        }

        if (gamepad1.a) flywheelRPM = 0;

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


        // COLOR SENSING + LED CONTROL
        double red = colorSensor.red();
        double green = colorSensor.green();
        double blue = colorSensor.blue();

        boolean colorDetected = false;

        if (isGreen(red, green, blue)) {
            setLEDs(GREEN_POS);
            lastSeenTime = System.currentTimeMillis();
            colorDetected = true;
            telemetry.addLine("Detected: Green");
        } else if (isPurple(red, green, blue)) {
            setLEDs(PURPLE_POS);
            lastSeenTime = System.currentTimeMillis();
            colorDetected = true;
            telemetry.addLine("Detected: Purple");
        }

        if (!colorDetected && System.currentTimeMillis() - lastSeenTime > 3000) {
            setLEDs(BLUE_POS);
            telemetry.addLine("No color detected for 3s → Blue");
        }

        telemetry.addData("Color - Red", red);
        telemetry.addData("Color - Green", green);
        telemetry.addData("Color - Blue", blue);

        /* ---------------- GENERAL TELEMETRY ---------------- */
        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.addData("Hood Position", hoodPosition);
        telemetry.addData("PID Error", flywheelControl.pid_controller.error);
        telemetry.addData("Motor Speed", flywheelControl.motor_speed_rpm);
        telemetry.addData("Feedforward", flywheelControl.feedforward_power);
        telemetry.addData("PID Power", flywheelControl.pid_power);
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Hood Pos", hood.getPosition());
        telemetry.update();
    }

    // helper stuff for color detection
    private void setLEDs(double position) {
        leftLED.setPosition(position);
        rightLED.setPosition(position);
    }

    private boolean isGreen(double red, double green, double blue) {
        return green > red * 1.3 && green > blue * 1.3 && green > 50;
    }

    private boolean isPurple(double red, double green, double blue) {
        return red > 60 && blue > 60 && green < (red + blue) / 3;
    }
}
