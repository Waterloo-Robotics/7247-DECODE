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
    private Servo hood;


    /* ---------- Modules & Sensors ---------- */
    private flywheelModule flywheelControl;
    private Limelight3A limelight;
    private LimelightProcessingModule llModule;

    /* ---------- Variables ---------- */
    private double hoodPosition = 0.0;


    /* start of module stuff */
    /* end of module stuff */
    private double flywheelRPM;

// hood control booleans
// end of hood control booleans
    private boolean intakeActive = false;

    @Override
    public void init() {
        /* ----- Hardware Map ----- */
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // end of hardware map stuff
        hood = hardwareMap.get(Servo.class, "hood");

        // Limelight initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Mecanum motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Modules
        flywheelControl = new flywheelModule(flywheel);
        flywheelRPM = 0;

        llModule = new LimelightProcessingModule(limelight, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void loop() {
        /* ---------------- DRIVE CODE ---------------- */
        double y = -gamepad1.left_stick_y;   // Forward/backward
        double x = gamepad1.left_stick_x;    // Strafe left/right
        double turn = gamepad1.right_stick_x; // Rotate

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
        double rightTrigger = gamepad1.right_trigger; // 0.0–1.0
        double leftTrigger = gamepad1.left_trigger;   // 0.0–1.0

        // Analog ramp based on trigger pressure
        flywheelRPM += rightTrigger * 50;   // faster press → faster ramp up
        flywheelRPM -= leftTrigger * 50;    // faster press → faster ramp down

        // Triangle to stop flywheel fast without triggers
        if (gamepad1.y) {
            flywheelRPM = 0;
        }

        // FLYWHEEL
        if (flywheelRPM < 0) flywheelRPM = 0;
        if (flywheelRPM > 4200) flywheelRPM = 4200;

        flywheelControl.set_speed((int) flywheelRPM);

        /* ---------------- INTAKE CONTROL ---------------- */
        if (gamepad1.right_bumper) intakeActive = true;
        if (gamepad1.left_bumper) intakeActive = false;
        intake.setPower(intakeActive ? 1.0 : 0.0);

        /* ---------------- HOOD CONTROL ---------------- */
        if (gamepad1.dpad_up && hoodPosition < 1.0) {
            hoodPosition += 0.0001;
        } else if (gamepad1.dpad_down && hoodPosition > 0.0) {
            hoodPosition -= 0.0001;
        }
        hood.setPosition(hoodPosition);
        // end of hood control stuff

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
        telemetry.addData("PID Error", flywheelControl.pid_controller.error);
        telemetry.addData("Motor Speed", flywheelControl.motor_speed_rpm);
        telemetry.addData("Feedforward", flywheelControl.feedforward_power);
        telemetry.addData("PID Power", flywheelControl.pid_power);
        telemetry.addData("Intake Active", intakeActive);
        telemetry.update();
    }
}
