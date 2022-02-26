package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;

public class RobotInput {
    private static final double JOYSTICK_DEADZONE = 0.05;
    private final PS4Controller ps4Controller;
    private final GenericHID extremeController;

    public RobotInput(PS4Controller ps4Controller, GenericHID extremeController) {
        this.ps4Controller = ps4Controller;
        this.extremeController = extremeController;
    }

    public double getForward() {
        if (!ps4Controller.isConnected()) {
            return 0.0;
        }
        double raw = -ps4Controller.getLeftY();
        if (Math.abs(raw) < JOYSTICK_DEADZONE) {
            return 0.0;
        }
        return raw;
    }
    public double getSteer() {
        if (!ps4Controller.isConnected()) {
            return 0.0;
        }
        double raw = ps4Controller.getRightX();
        if (Math.abs(raw) < JOYSTICK_DEADZONE) {
            return 0.0;
        }
        return raw;
    }

    public double getIntakeSpeed() {
        if (!extremeController.isConnected()) {
            return 0.0;
        }
        int pov = extremeController.getPOV(Constants.ControllerExtreme.POV);
        if (pov == -1) {
            return 0.0;
        }
        if (pov == 0 || pov == 45 || pov == 315) { // up or diagonal up
            return -1.0; // if POV is up, spit out
        }
        if (pov == 90 || pov == 270) { // side
            return 0.0;
        }
        return 1.0;
    }

    /**
     * Assuming the labels are kept on the extreme joystick, this just 1.0 for in and -1.0 for out indexer
     */
    public double getManualIndexerSpeed() {
        double value = 0.0;
        if (isGridMiddleLeft()) { // in
            value += 1.0;
        }
        if (isGridMiddleRight()) { // out
            value -= 1.0;
        }
        return value;
    }


    private boolean isGridMiddleLeft() {
        return extremeController.isConnected() && extremeController.getRawButton(Constants.ControllerExtreme.GRID_MIDDLE_LEFT);
    }
    private boolean isGridMiddleRight() {
        return extremeController.isConnected() && extremeController.getRawButton(Constants.ControllerExtreme.GRID_MIDDLE_RIGHT);
    }
}
