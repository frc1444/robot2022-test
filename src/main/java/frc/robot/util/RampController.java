package frc.robot.util;

import edu.wpi.first.math.MathUtil;

public class RampController {

    private final double rampRate;
    private final double period;

    private double setpoint = 0.0;

    /**
     *
     * @param rampRate The ramp rate. (percent per second)
     * @param period
     */
    public RampController(double rampRate, double period) {
        this.rampRate = rampRate;
        this.period = period;
    }


    public void updateWithDesired(double desiredSetpoint) {
        desiredSetpoint = MathUtil.clamp(desiredSetpoint, -1.0, 1.0);
        double difference = desiredSetpoint - setpoint;
        double toAdd = period * rampRate;
        if (Math.abs(difference) < toAdd) {
            setpoint = desiredSetpoint;
            return;
        }
        setpoint = setpoint + Math.signum(difference) * toAdd;
    }
    public double getSetpoint() {
        return setpoint;
    }

    public void reset() {
        setpoint = 0.0;
    }
}
