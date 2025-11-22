package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.PIDController;

@Autonomous(name = "autoTest", group = "Auto")
public class autoTest extends LinearOpMode {

    DcMotor backLeft, backRight, frontLeft, frontRight;
    DcMotorEx flywheel;
    DcMotor intake, transfer;
    Servo hood;

    @Override
    public void runOpMode() {

        // Hardware
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        hood = hardwareMap.get(Servo.class, "hood");

        // Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        // PID
        PIDController pid = new PIDController(0.00085f, 0.000002f, 0.00015f);
        pid.set_output_limits(0, 1);  // power 0–1
        pid.set_tolerance(50);

        waitForStart();

        // hood
        hood.setPosition(0.7);
        sleep(300);

        // Short backward drive
        setDrivePower(-0.4);
        sleep(500);
        setDrivePower(0);

        // FLYWHEEL PID -----------------
        int targetRPM = 2700;
        pid.set_target(targetRPM);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Spin-up for 3 seconds
        long end = System.currentTimeMillis() + 2200;
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            double currentRPM = (flywheel.getVelocity() / 28.0) * 60.0;
            double power = pid.update(currentRPM);
            flywheel.setPower(power);
        }

        shootBall(450);
        sleep(350);

        shootBall(450);
        sleep(350);

        shootBall(850);

        // stop flywheel
        sleep(500);
        flywheel.setPower(0);
    }

    private void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void shootBall(int time) {
        intake.setPower(1);
        transfer.setPower(-1);
        sleep(time);
        intake.setPower(0);
        transfer.setPower(0);
    }
}
