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
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.modules.FCDrivebaseModule;

import org.firstinspires.ftc.teamcode.modules.LauncherModule;
import org.firstinspires.ftc.teamcode.modules.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.modules.Table2D;
import org.firstinspires.ftc.teamcode.modules.flywheelModule;

@TeleOp(name="BioBuzzPrep Teleop", group="LinearOpMode")
public class BioBuzzPrep_TeleOp extends OpMode {

    /* ---------- Drive Motors ---------- */
    public DcMotor intakeMotor;
    public DcMotor passthroughMotor1;
    public DcMotor passthroughMotor2;


    /* ---------- Modules & Sensors ---------- */


    /* ---------- Variables ---------- */


    @Override
    public void init() {
        /* ----- Hardware Map ----- */
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        passthroughMotor1 = hardwareMap.get(DcMotor.class, "passthroughMotor1");
        passthroughMotor2 = hardwareMap.get(DcMotor.class, "passthroughMotor2");

        // Mecanum motor directions


        // Modules


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start() {

    }

    public void loop() {

    }

    public void launcher() {
        //Start intake while the A button on Gamepad 2 is pressed
        if (gamepad2.a) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }

        //Start passthrough while the B button on Gamepad 2 is pressed
        if (gamepad2.b) {
            passthroughMotor1.setPower(1);
            passthroughMotor2.setPower(1);
        } else {
            passthroughMotor1.setPower(0);
            passthroughMotor2.setPower(0);
        }
        (Call_WinstonsAunt_From_Couch);
    }
}
