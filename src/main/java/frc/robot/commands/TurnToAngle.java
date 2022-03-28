package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends PIDCommand {
    public TurnToAngle(double angle, DriveSubsystem drive) {
        super(
            createPIDController(),
            drive::getHeading,
            angle,
            output -> drive.curvatureDrive(0.0, output),
            drive
        );
    }
    /** Creates a {@link PIDController} desired to be used with degrees */
    public static PIDController createPIDController() {
        PIDController pidController = new PIDController(
            Constants.DriveConstants.TURN_KP,
            Constants.DriveConstants.TURN_KI,
            Constants.DriveConstants.TURN_KD,
            Constants.PERIOD
        );
        // Set the controller to be continuous (because it is an angle controller)
        pidController.enableContinuousInput(-180, 180);

        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        pidController.setTolerance(
            Constants.DriveConstants.TURN_TOLERANCE_DEG,
            Constants.DriveConstants.TURN_RATE_TOLERANCE
        );
        return pidController;
    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    protected static double calculateSetpoint(double angle) {
        return Math.IEEEremainder(angle, 360.0);
    }
    
}
