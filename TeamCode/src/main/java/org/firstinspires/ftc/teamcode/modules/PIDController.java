package org.firstinspires.ftc.teamcode.modules;

public class PIDController {
    public double target;
    public double current;
    public double error;
    private double last_error;

    /* PID Constants */
    private float kP;
    private float kI;
    private float kD;

    /* Output Terms */
    private double pTerm;
    private double iTerm;
    private double dTerm;

    /* Configuration */
    private double tolerance;
    private double max_output = 1;
    private double min_output = -1;

    /* Locals */
    private boolean limited;

    public PIDController(float p, float i, float d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public void set_target(float target) {
        this.target = target;
    }

    public void set_tolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double update(double current_position) {
        this.error = this.target - current_position;

        this.pTerm = this.error * this.kP;

        /* Only add to the integrator if we're not saturated */
        if (!this.limited && Math.signum(this.iTerm) == Math.signum(this.error)) {
            this.iTerm = iTerm + (this.error * this.kI);
        }

        this.dTerm = (this.error - this.last_error) * this.kD;

        double raw_output = this.pTerm + this.iTerm + this.dTerm;
        double bound_output;

        /* Bound raw output with min and max output */
        if (raw_output < this.min_output) {
            this.limited = true;
            bound_output = this.min_output;
        } else if (raw_output > this.max_output) {
            this.limited = true;
            bound_output = this.max_output;
        } else {
            this.limited = false;
            bound_output = raw_output;
        }

        return bound_output;
    }

    public boolean at_target() {
        return (Math.abs(this.error) < this.tolerance);
    }

    public void set_output_limits(double min, double max) {
        this.min_output = min;
        this.max_output = max;
    }

}
