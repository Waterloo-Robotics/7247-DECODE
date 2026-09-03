package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.modules.FCDrivebaseModule;
import org.firstinspires.ftc.teamcode.modules.IndexerModule;
import org.firstinspires.ftc.teamcode.modules.LimelightProcessingModule;
import org.firstinspires.ftc.teamcode.modules.Table2D;
import org.firstinspires.ftc.teamcode.modules.TurretModule;
import org.firstinspires.ftc.teamcode.modules.flywheelModule;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="BioBuzzMaster", group="LinearOpMode")
public class BioBuzzMaster extends OpMode {

    /* ---------- Drive Motors ---------- */
    public DcMotor motorFL;
    public DcMotor motorBL;
    public DcMotor motorFR;
    public DcMotor motorBR;

    /* ---------- Variables ---------- */
    public float y_power;
    public float x_power; 
    public float turn_power;

    @Override
    public void init() {
        /* ----- Hardware Map ----- */
        motorFL = hardwareMap.get(DcMotor.class, "frontLeft");
        motorBL = hardwareMap.get(DcMotor.class, "backLeft");
        motorFR = hardwareMap.get(DcMotor.class, "frontRight");
        motorBR = hardwareMap.get(DcMotor.class, "backRight");


        // Mecanum motor directions
        

        // Modules
        

    }

    @Override
    public void loop() 
    {
        driveBase();
    }

    public void driveBase()
    {
        y_power = gamepad1.left_stick_y;
        x_power = gamepad1.left_stick_x;
        turn_power = gamepad1.right_stick_x;

        motorBL.setPower(y_power + turn_power - x_power); 
        motorFL.setPower(y_power + turn_power + x_power);
        motorBR.setPower(y_power - turn_power + x_power);
        motorFR.setPower(y_power - turn_power - x_power);
    }
}