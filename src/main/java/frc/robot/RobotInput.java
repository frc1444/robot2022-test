package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotInput {

    private final PS4Controller _ps4Controller;
    private final GenericHID _extremeController;

    public RobotInput(PS4Controller ps4Controller, GenericHID extremeController) {
        _ps4Controller = ps4Controller;
        _extremeController = extremeController;
    }

    public JoystickButton getIntakeButton() {
        return new JoystickButton(_ps4Controller, PS4Controller.Button.kTriangle.value);
    }

    public JoystickButton getEjectButton() {
        return new JoystickButton(_ps4Controller, PS4Controller.Button.kCircle.value);
    }

    public JoystickButton getEjectLowerButton() {
        return new JoystickButton(_ps4Controller, PS4Controller.Button.kCross.value);
    }

    public JoystickButton getShootLowButton() {
        return new JoystickButton(_ps4Controller, PS4Controller.Button.kSquare.value);
    }

    public JoystickButton getFeedButton() {
        return new JoystickButton(_ps4Controller, PS4Controller.Button.kR1.value);
    }

    public double getForward() {
        if (!_ps4Controller.isConnected()) {
            return 0.0;
        }
        double raw = -_ps4Controller.getLeftY();
        if (Math.abs(raw) < Constants.DRIVE_JOYSTICK_DEADZONE) {
            return 0.0;
        }

        return applyInputCurve(raw, Constants.FORWARD_INPUT_CURVE);
    }
    public double getSteer() {
        if (!_ps4Controller.isConnected()) {
            return 0.0;
        }
        double raw = _ps4Controller.getRightX();
        if (Math.abs(raw) < Constants.DRIVE_JOYSTICK_DEADZONE) {
            return 0.0;
        }
        return applyInputCurve(raw, Constants.ROTATE_INPUT_CURVE);
    }

    public double getIntakeSpeed() {
        if (!_extremeController.isConnected()) {
            return 0.0;
        }
        int pov = _extremeController.getPOV(Constants.ControllerExtreme.POV);
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
        return _extremeController.isConnected() && _extremeController.getRawButton(Constants.ControllerExtreme.GRID_MIDDLE_LEFT);
    }
    private boolean isGridMiddleRight() {
        return _extremeController.isConnected() && _extremeController.getRawButton(Constants.ControllerExtreme.GRID_MIDDLE_RIGHT);
    }

    /**
     * Apply polynomial curve to input. This can be used to make controls more sensitive at lower speeds
     * @param raw The raw input to be modified
     * @param power Power of the polynomial to be applied.
     * @return The modified input
     */
    private double applyInputCurve(double raw, double power) {

        // Avoid taking the input to a negative or zero power
        if (power <= 0) {
            power = 1;
        }

        return Math.signum(raw) * Math.pow(Math.abs(raw), power);
    }
}
