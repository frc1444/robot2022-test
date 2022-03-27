package frc.robot.vision;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class VisionState {
    private static final double DIM_TIME = 1.0;
    private static final double MAX_BRIGHTNESS = .75;

    private final DigitalOutput pwm;

    private double dimStartTime = 0;
    private boolean enabled;

    public VisionState(){
        pwm = new DigitalOutput(Constants.DigitalIO.VISION_LED);
        pwm.enablePWM(1.0);
        pwm.setPWMRate(500);
    }

    public void update() {
        if(enabled){
            pwm.updateDutyCycle(MAX_BRIGHTNESS);
        } else {
            double elapsed = Timer.getFPGATimestamp() - dimStartTime;
            double percent = Math.max(0, 1 - elapsed / DIM_TIME);
            percent = Math.pow(percent, 2);
            pwm.updateDutyCycle(percent * MAX_BRIGHTNESS);
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void setEnabled(boolean enabled) {
        if(this.enabled && !enabled){
            dimStartTime = Timer.getFPGATimestamp();
        }
        this.enabled = enabled;
    }
}
