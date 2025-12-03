package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.modules.FieldPositionEstimation;
import org.firstinspires.ftc.teamcode.modules.LimelightProcessingModule;

@TeleOp(name = "fuseTest")
public class fuseTest extends OpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    private GoBildaPinpointDriver pinpoint;
    private FieldPositionEstimation estimator;
    private Limelight3A limelight;
    private LimelightProcessingModule llModule;

    private Pose2D lastValidTagPose = null;

    @Override
    public void init() {
        // Motors
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Pinpoint + Estimator
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        estimator = new FieldPositionEstimation(pinpoint, false);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llModule = new LimelightProcessingModule(limelight, telemetry);
        estimator.attachLimelight(limelight);
    }

    @Override
    public void loop() {
        // === DRIVETRAIN ===
        double y = -gamepad1.left_stick_y * 0.6;
        double x = gamepad1.left_stick_x * 0.6;
        double turn = gamepad1.right_stick_x * 0.6;

        double fl = y + x + turn;
        double fr = y - x - turn;
        double bl = y - x + turn;
        double br = y + x - turn;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        // === UPDATE SENSORS ===
        estimator.update_from_pinpoint();

        // Use your proven LimelightProcessingModule to get global pose (0,0 = field center)
        Pose2D tagPose = llModule.getFieldSpacePose();  // This is already field-space, correct origin

        if (tagPose != null && gamepad1.aWasPressed()) {
            lastValidTagPose = tagPose;

            // Correct Pinpoint to global field pose when tag is seen
            pinpoint.setPosition(new Pose2D(
                    DistanceUnit.INCH,
                    tagPose.getX(DistanceUnit.INCH),
                    tagPose.getY(DistanceUnit.INCH),
                    AngleUnit.DEGREES,
                    tagPose.getHeading(AngleUnit.DEGREES)
            ));
        }

        // Let estimator handle fusion (it will use tag pose if available via internal update)
        estimator.update_from_limelight();

        Pose2D odoPose = estimator.relative_robot_position;
        Pose2D fusedPose = estimator.getFusedFieldPosition();

        // === TELEMETRY ===
        telemetry.addLine("--- Odometry (Pinpoint) ---");
        telemetry.addData("X", "%.2f in", odoPose.getX(DistanceUnit.INCH));
        telemetry.addData("Y", "%.2f in", odoPose.getY(DistanceUnit.INCH));
        telemetry.addData("H", "%.2f°", odoPose.getHeading(AngleUnit.DEGREES));

        if (tagPose != null) {
            telemetry.addLine("--- AprilTag (Global) ---");
            telemetry.addData("X", "%.2f in", tagPose.getX(DistanceUnit.INCH));
            telemetry.addData("Y", "%.2f in", tagPose.getY(DistanceUnit.INCH));
            telemetry.addData("H", "%.2f°", tagPose.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Status", "VISIBLE → Pinpoint Corrected");
        } else if (lastValidTagPose != null) {
            telemetry.addData("Last Tag X", "%.2f in", lastValidTagPose.getX(DistanceUnit.INCH));
            telemetry.addData("Last Tag Y", "%.2f in", lastValidTagPose.getY(DistanceUnit.INCH));
        }

        telemetry.addLine("--- FUSED POSE (Field Coordinates) ---");
        telemetry.addData("X", "%.2f in", fusedPose.getX(DistanceUnit.INCH));
        telemetry.addData("Y", "%.2f in", fusedPose.getY(DistanceUnit.INCH));
        telemetry.addData("H", "%.2f°", fusedPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Source", tagPose != null ? "Limelight" : "Odometry");

        telemetry.update();
    }
}