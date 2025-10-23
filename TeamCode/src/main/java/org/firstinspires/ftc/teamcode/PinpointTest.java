package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "pinpointTest", group="LinearOpMode")
public class PinpointTest extends OpMode {

    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private GoBildaPinpointDriver Tpinpoint;
    private FieldPositionEstimation field_estimator;

    @Override
    public void init() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        Tpinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        field_estimator = new FieldPositionEstimation(Tpinpoint, false);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);


    }
    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y *.6;   // Forward/backward
        double x = gamepad1.left_stick_x *.6;    // Strafe left/right
        double turn = gamepad1.right_stick_x*.6; // Rotate in place

        // Calculate motor powers for mecanum drive
        double frontLeftPower = y + x + turn;
        double frontRightPower = y - x - turn;
        double backLeftPower = y - x + turn;
        double backRightPower = y + x - turn;

        // Normalize motor powers to stay within [-1.0, 1.0]
        double max = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }


        // Set powers to motors
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        boolean reset= gamepad1.a;
        if(reset){
            field_estimator.reset_pinpoint();
        }
        field_estimator.update_from_pinpoint();


        telemetry.addData("Position X", "%f.2", field_estimator.relative_robot_position.getX(DistanceUnit.INCH));
        telemetry.addData("Position Y", "%f.2", field_estimator.relative_robot_position.getY(DistanceUnit.INCH));
        telemetry.addData("Rotation", "%f.2", field_estimator.relative_robot_position.getHeading(AngleUnit.DEGREES));
    }

}
