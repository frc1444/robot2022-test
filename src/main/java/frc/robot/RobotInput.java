package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotInput {

    private final PS4Controller _driveController;
    private final PS4Controller _operatorController;

    private boolean _singleControllerMode;

    public RobotInput(PS4Controller ps4Controller, PS4Controller operatorController) {
        _driveController = ps4Controller;
        _operatorController = operatorController;

        _singleControllerMode = !_operatorController.isConnected();
    }

    public Trigger getIntakeButton() {
        return new JoystickButton(_singleControllerMode ? _driveController : _operatorController, 
            PS4Controller.Button.kTriangle.value);     
    }

    public Trigger getEjectButton() {
        return new JoystickButton(_singleControllerMode ? _driveController : _operatorController,
            PS4Controller.Button.kCross.value);
    }

    public Trigger getEjectLowerButton() {
        return new JoystickButton(_singleControllerMode ? _driveController : _operatorController,
            PS4Controller.Button.kCircle.value);
    }

    public Trigger getShootLowButton() {
        return new JoystickButton(_singleControllerMode ? _driveController : _operatorController,
            PS4Controller.Button.kL1.value);
    }

    public Trigger getShootHighButton() {
        return new JoystickButton(_singleControllerMode ? _driveController : _operatorController,
            PS4Controller.Button.kR1.value);
    }

    public Trigger getRaiseIntake() {
        return new Trigger(_singleControllerMode ? this::driverPovEquals0 : this::operatorPovEquals0);
    }

    public Trigger getLowerIntake() {
        return new Trigger(_singleControllerMode ? this::driverPovEquals180 : this::operatorPovEquals180);
    }

    public Trigger getStopButton() {
        return new JoystickButton(_singleControllerMode ? _driveController : _operatorController,
            PS4Controller.Button.kSquare.value);
    }

    public Trigger getShiftHigh() {
        return new JoystickButton(_driveController, 
            _singleControllerMode ? PS4Controller.Button.kR3.value : PS4Controller.Button.kR1.value);
    }

    public Trigger getShiftLow() {
        return new JoystickButton(_driveController, 
            _singleControllerMode ? PS4Controller.Button.kL3.value : PS4Controller.Button.kL1.value);
    }

    public Trigger getQuickLeft() {
        // If single controller mode, there won't be quick move triggers, so just return 0
        return new JoystickButton(_driveController, 
            _singleControllerMode ? 0 : PS4Controller.Button.kSquare.value);
    }

    public Trigger getQuickRight() {
        // If single controller mode, there won't be quick move triggers, so just return 0
        return new JoystickButton(_driveController, 
            _singleControllerMode ? 0 : PS4Controller.Button.kCircle.value);
    }

    public double getForward() {
        if (!_driveController.isConnected()) {
            return 0.0;
        }
        double raw = -_driveController.getLeftY();
        //if (Math.abs(raw) < Constants.DRIVE_JOYSTICK_DEADZONE) {
        //    return 0.0;
        //}

        return applyInputCurve(raw, Constants.FORWARD_INPUT_CURVE);
    }
    public double getSteer() {
        if (!_driveController.isConnected()) {
            return 0.0;
        }
        double raw = _driveController.getRightX();
        //if (Math.abs(raw) < Constants.DRIVE_JOYSTICK_DEADZONE) {
         //   return 0.0;
        //}

        return applyInputCurve(raw, Constants.ROTATE_INPUT_CURVE);
    }

    private boolean driverPovEquals0() {
        return _driveController.getPOV() == 0;
    }

    private boolean driverPovEquals180() {
        return _driveController.getPOV() == 180;
    }

    private boolean operatorPovEquals0() {
        return _operatorController.getPOV() == 0;
    }

    private boolean operatorPovEquals180() {
        return _operatorController.getPOV() == 180;
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
