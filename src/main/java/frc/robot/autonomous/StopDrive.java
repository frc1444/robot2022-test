package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class StopDrive extends CommandBase {
    
    public final DriveSubsystem _drive;

    public StopDrive (DriveSubsystem drive) {
        _drive = drive;
    }

    @Override
    public void initialize() {
        _drive.tankDriveVolts(0.0,0.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
