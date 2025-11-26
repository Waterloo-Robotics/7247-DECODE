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

@TeleOp(name="fuseTest", group="Localization")
public class fuseTest extends OpMode {

    private DcMotor backLeft, backRight, frontLeft, frontRight;

    private GoBildaPinpointDriver pinpoint;
    private FieldPositionEstimation estimator;

    private Limelight3A limelight;
    private LimelightProcessingModule llModule;

    private Pose2D fusedPose;

    @Override
    public void init() {

        // ---- MOTORS ----
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // ---- PINPOINT ----
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        estimator = new FieldPositionEstimation(pinpoint, false);

        // ---- LIMELIGHT ----
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        llModule = new LimelightProcessingModule(limelight, telemetry);

        fusedPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    }

    @Override
    public void loop() {

        // ---------------------------------------------------
        // 1. driving
        // ---------------------------------------------------
        double y = -gamepad1.left_stick_y * 0.6;
        double x =  gamepad1.left_stick_x * 0.6;
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


        // ---------------------------------------------------
        // 2. pinpoint
        // ---------------------------------------------------
        estimator.update_from_pinpoint();
        Pose2D odoPose = estimator.relative_robot_position;


        // ---------------------------------------------------
        // 3. limelight
        // ---------------------------------------------------
        Pose2D tagPose = llModule.limelightResult();  // null if no tag


        // ---------------------------------------------------
        // 4. fusion - might not work yet
        // ---------------------------------------------------
        if (tagPose != null) {

            fusedPose = tagPose;

            // THIS IS THE FIX:
            // Pinpoint expects ONE Pose2D argument.
            pinpoint.setPosition(
                    new Pose2D(
                            DistanceUnit.INCH,
                            tagPose.getX(DistanceUnit.INCH),
                            tagPose.getY(DistanceUnit.INCH),
                            AngleUnit.DEGREES,
                            tagPose.getHeading(AngleUnit.DEGREES)
                    )
            );

            telemetry.addLine("Tag seen → Pinpoint corrected");
        } else {
            fusedPose = odoPose;
        }


        // ---------------------------------------------------
        // 5. TELEMETRY
        // ---------------------------------------------------
        telemetry.addLine("--- Pinpoint Raw ---");
        telemetry.addData("Odo X", odoPose.getX(DistanceUnit.INCH));
        telemetry.addData("Odo Y", odoPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", odoPose.getHeading(AngleUnit.DEGREES));

        if (tagPose != null) {
            telemetry.addLine("--- AprilTag ---");
            telemetry.addData("Tag X", tagPose.getX(DistanceUnit.INCH));
            telemetry.addData("Tag Y", tagPose.getY(DistanceUnit.INCH));
            telemetry.addData("Tag Heading", tagPose.getHeading(AngleUnit.DEGREES));
        }

        telemetry.addLine("--- FUSED POSE ---");
        telemetry.addData("X", fusedPose.getX(DistanceUnit.INCH));
        telemetry.addData("Y", fusedPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", fusedPose.getHeading(AngleUnit.DEGREES));

        telemetry.update();

    }
}
