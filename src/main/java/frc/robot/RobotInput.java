package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;

public class RobotInput {
    private static final double JOYSTICK_DEADZONE = 0.05;
    private final PS4Controller ps4Controller;

    public RobotInput(PS4Controller ps4Controller) {
        this.ps4Controller = ps4Controller;
    }

    public double getForward() {
        double raw = -ps4Controller.getLeftY();
        if (Math.abs(raw) < JOYSTICK_DEADZONE) {
            return 0.0;
        }
        return raw;
    }
    public double getSteer() {
        double raw = ps4Controller.getRightX();
        if (Math.abs(raw) < JOYSTICK_DEADZONE) {
            return 0.0;
        }
        return raw;
    }
}
