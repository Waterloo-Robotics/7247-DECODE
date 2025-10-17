package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class flywheelModule {

    public DcMotor flywheel;
    public float output_power;
    public float feedforward_power;
    public float pid_power;

    public flywheelModule(DcMotor dcMotor)
    {
        this.flywheel = dcMotor;
    }

    public void set_speed(int speed_rpm)
    {
        float calc_power = ((float)speed_rpm) / 41 / 100;
        float limit_1 = Math.min(calc_power, 1);
        float limit_neg_1 = Math.max(limit_1, -1);
        this.feedforward_power = limit_neg_1;

        this.output_power = this.feedforward_power + this.pid_power;
        this.flywheel.setPower(this.output_power);
    }
}

