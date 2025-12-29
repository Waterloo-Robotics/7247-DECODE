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

@TeleOp(name="H2OLooBots_Final_Bot", group="LinearOpMode")
public class H2OLooBots_Final_Bot extends OpMode {

    /* ---------- Drive Motors ---------- */
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;

    private DcMotor backIntake;

    private DcMotor frontIntake;
    FCDrivebaseModule drivebase;
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        /* ----- Hardware Map ----- */
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");  // ORANGE & port 1 on EXPANTION hub
        backRight = hardwareMap.get(DcMotor.class, "backRight"); // GREEN & port 2 on CONTROL hub
        frontRight = hardwareMap.get(DcMotor.class, "frontRight"); //BLUE & port 1 on CONTOL hub
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");  // RED  & port 0 on EXPANTION hub
        backIntake= hardwareMap.get(DcMotor.class, "backIntake"); // YELLOW & port 2 on EXPANTION hub
        frontIntake= hardwareMap.get(DcMotor.class, "frontIntake"); // PURPLE & port 0 on CONTROL
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint"); // 12c Bus 0 on CONTROL hub


        drivebase = new FCDrivebaseModule(backLeft, backRight, frontLeft, frontRight, pinpoint);

        // Mecanum motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {


        /* ---------------- DRIVE CODE ---------------- */
        pinpoint.update();
        drivebase.update_Drive(gamepad1.left_stick_x,-gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (gamepad1.options) {
            pinpoint.update();
            pinpoint.resetPosAndIMU();
        }

    }
}