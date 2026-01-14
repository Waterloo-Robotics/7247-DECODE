package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterModule {
    private Servo hood_servo;
    private DcMotor flywheel_motor;



    public ShooterModule(Servo hood_servo, DcMotor flywheel_motor)
    {
        this.hood_servo = hood_servo;
        this.flywheel_motor = flywheel_motor;
    }


}
