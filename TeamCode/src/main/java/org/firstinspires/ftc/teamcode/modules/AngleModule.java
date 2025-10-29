package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class AngleModule {
    public PIDController pid_controller;

public AngleMod(void)
{
    this.pid_controller = new PIDController((float) 0.002, 0, 0);
}
    public double set_Angle( double angle, double current_angle)
    {

        // Update PID Controller
        this.pid_controller.set_target((float)angle);
        return this.pid_controller.update((float)current_angle);
    }
}


