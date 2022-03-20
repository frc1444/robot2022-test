package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class TurnToAngle extends PIDCommand {
    public TurnToAngle(double angle, DriveSubsystem drive) {
        super(
            new PIDController(0.0005, 0.0, 0.0),
            drive::getHeading,
            angle,
            output -> drive.arcadeDrive(0.0, output),
            drive
        );

        // Set the controller to be continuous (because it is an angle controller)
        getController().enableContinuousInput(-180, 180);
    
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        getController().setTolerance(5.0, 10.0);

    }    

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

    protected static double calculateSetpoint(double angle) {
        return Math.IEEEremainder(angle, 360.0);
    }
    
}
