package org.firstinspires.ftc.teamcode.modules;

public class PIDController {
    public double target;
    public double current;
    public double error;

    public float kP;
    public float kI;
    public float kD;

    private double tolerance;

    public PIDController(float p, float i, float d)
    {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public void set_target(float target)
    {
        this.target = target;
    }

    public void set_tolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }

    public double update(double current_position)
    {
        // Calculate error
        this.error = this.target - current_position;

        double pTerm = this.error * this.kP;
        float iTerm = 0;
        float dTerm = 0;

        return pTerm + iTerm + dTerm;
    }

    public boolean at_target()
    {
        return (Math.abs(this.error) < this.tolerance);
    }

}
