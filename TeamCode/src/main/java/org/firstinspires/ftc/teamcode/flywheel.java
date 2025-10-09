package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class flywheel extends OpMode {

    private DcMotor flywheel;

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        flywheel.setPower(0.9);
    }

}
