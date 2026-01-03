package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "redAuto", group = "Auto")
public class redAuto extends LinearOpMode {

    private DcMotor backLeft, backRight, frontLeft, frontRight;


    @Override
    public void runOpMode() {

        // Hardware mapping
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        // Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {


            sleep(1000);
            frontLeft.setPower(-0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(-0.5);
            backRight.setPower(0.5);
            sleep(500);



        }
    }
}
