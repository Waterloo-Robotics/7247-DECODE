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

@TeleOp(name="BioBuzzPrep Teleop", group="LinearOpMode")
public class BioBuzzPrep_TeleOp extends OpMode {

    /* ---------- Drive Motors ---------- */


    /* ---------- Modules & Sensors ---------- */


    /* ---------- Variables ---------- */


    @Override
    public void init() {
        /* ----- Hardware Map ----- */


        // Mecanum motor directions


        // Modules


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    public void start() 
    {

    }

    public void loop() 
    {

    }
}
