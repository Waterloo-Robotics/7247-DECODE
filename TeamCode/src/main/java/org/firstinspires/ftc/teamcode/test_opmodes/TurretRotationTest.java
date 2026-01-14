package org.firstinspires.ftc.teamcode.test_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.flywheelModule;

@TeleOp(name="Turret Rotation Test", group="TestOpMode")
public class TurretRotationTest extends OpMode {
    private DcMotor TurretR;
    private Servo hood;



    @Override
    public void init(){
        TurretR = hardwareMap.get(DcMotor.class, "turretRotation");
        hood = hardwareMap.get(Servo.class, "hood");
    }
    public void init_loop() {
    }
//90 degree =389 motor clicks

    @Override
    public void start() {
    }

    @Override
    public void loop() {

if(gamepad1.a){
    TurretR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
}
        TurretR.setPower(gamepad1.left_stick_y);

        telemetry.addData("Turret Count", TurretR.getCurrentPosition());
    }
}

