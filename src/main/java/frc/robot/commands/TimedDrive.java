package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDrive  extends CommandBase {
    
    DriveSubsystem _drive;

    public TimedDrive(DriveSubsystem drive) {
        _drive = drive;
    }

    @Override
    public void initialize() {
        _drive.curvatureDrive(-0.25, 0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
