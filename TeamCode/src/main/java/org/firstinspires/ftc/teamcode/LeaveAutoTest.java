package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Leave Auto", group="Linear OpMode")
public class LeaveAutoTest extends LinearOpMode {
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private ElapsedTime runtime = new ElapsedTime();

    static final double FORWARD_POWER = 0.5; /*power sent to the motors,
    we can adjust this as needed */
    static final long DRIVE_TIME_MS = 3000; // can also be adjusted

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        if (opModeIsActive()) {
            frontLeft.setPower(FORWARD_POWER);
            frontRight.setPower(FORWARD_POWER);
            backLeft.setPower(FORWARD_POWER);
            backRight.setPower(FORWARD_POWER);
        }
        while (opModeIsActive() && runtime.milliseconds() < DRIVE_TIME_MS) {
            telemetry.addLine("Driving");
            telemetry.update();
        }
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        telemetry.addLine("Done Driving, Ready for TeleOp!");
        telemetry.update();
    }
}
