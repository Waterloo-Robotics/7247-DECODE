package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.modules.FCDrivebaseModule;
import org.firstinspires.ftc.teamcode.modules.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.modules.Table2D;
import org.firstinspires.ftc.teamcode.modules.flywheelModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

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

    FCDrivebaseModule drivebase;

    float[] distance = {22, 30, 35, 40,44,52,56,69,81,125,126};
    private float[] flywheel_speed = {2650, 2900, 3000, 3100, 3150, 3270, 3300, 3250, 3350, 4000,4000};
    private float[] hood_angle = { (float)0.75, (float)0.75, (float)0.75, (float)0.75, (float)0.75,(float)0.75,(float)0.75,(float)0.75,(float)0.65,(float)0.55, (float)0.50};
    private Table2D flywheel_speed_table = new Table2D(distance, flywheel_speed);
    private Table2D hood_angle_table = new Table2D(distance, hood_angle);
    boolean AutoTargeting;
    GoBildaPinpointDriver pinpoint;

    /* ---------- Modules & Sensors ---------- */
    private flywheelModule flywheelControl;
    private Limelight3A limelight;
    private LimelightProcessingModule llModule;

    /* ---------- Variables ---------- */
    private double hoodPosition = 0.4; // start in mid position
    private double flywheelRPM;

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
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();

        drivebase = new FCDrivebaseModule(backLeft, backRight, frontLeft, frontRight, pinpoint);

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
        float rpm = 0;
        float angle = 1;
        boolean limelight_available = false;

        Pose2D pose = llModule.limelightResult();

        float limelight_distance = 0;
        if (pose != null) {
            limelight_distance = (float) (1.75*(float) -pose.getX(DistanceUnit.INCH));

            if (limelight_distance < 81 || limelight_distance > 124) {
                limelight_available = true;
            }

            rpm =  (flywheel_speed_table.Lookup(limelight_distance));
            angle = hood_angle_table.Lookup(limelight_distance);
        }


        /* ---------------- DRIVE CODE ---------------- */
        pinpoint.update();
        drivebase.update_Drive(gamepad1.left_stick_x,-gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.options) {
            pinpoint.update();
            pinpoint.resetPosAndIMU();
        }
//

        /* ---------------- FLYWHEEL CONTROL ---------------- */
        flywheelRPM += gamepad2.right_trigger * 50;
        flywheelRPM -= gamepad2.left_trigger * 50;

        flywheelRPM = Math.max(0, Math.min(4200, flywheelRPM));
        flywheelControl.set_speed((int) flywheelRPM);

        /* ---------------- BALL / INTAKE / TRANSFER CONTROL ---------------- */
        double intakePower = 0.0;
        double transferPower = 0.0;

        // --- Touchpad reverses both while held ---
        if (gamepad2.touchpad || gamepad1.touchpad) {
            intakePower = -1.0;      // reverse
            transferPower = 1.0;    // reverse
            flywheelRPM = 3500;
        }

        // --- Ball 1 (B) ---
        else if (gamepad2.b || gamepad1.b) {
            intakePower = 1.0;     // forward
            transferPower = -1.0;   // forward
        }

        // --- Ball 2 (X) ---
        else if (gamepad2.x || gamepad1.x) {
            intakePower = 1.0;      // forward
        }

        // --- Default: stop both ---
        else {
            intakePower = 0.0;
            transferPower = 0.0;
        }

        // --- Flywheel Stop (A) ---
        if (gamepad2.a) {
            flywheelRPM = 0;
        }

        // Apply powers
        intake.setPower(intakePower);
        transfer.setPower(transferPower);
        flywheelControl.set_speed((int) flywheelRPM);

        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Transfer Power", transferPower);

        /* ---------------- HOOD CONTROL ---------------- */
        if (gamepad2.dpad_up) {
            hoodPosition -= 0.005;
        } else if (gamepad2.dpad_down) {
            hoodPosition += 0.005;
        }
        hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));

        if(gamepad2.dpadDownWasPressed() || gamepad1.dpadDownWasPressed()){
            AutoTargeting = !AutoTargeting;
        }
        if(AutoTargeting) {
            flywheelRPM = rpm;
            hoodPosition = angle;
        }

        if (gamepad2.dpad_left) {
            hoodPosition = 0.725;
            flywheelRPM = 3500;
        }

        if (gamepad2.dpad_right) {
            hoodPosition = 0.575;
            flywheelRPM = 3750;
        }

        hood.setPosition(hoodPosition);
        flywheelControl.set_speed((int) flywheelRPM);

        /* ---------------- LIMELIGHT TELEMETRY ---------------- */


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
        telemetry.addLine("Left = Short | Right = Far");
        telemetry.update();
    }
}
