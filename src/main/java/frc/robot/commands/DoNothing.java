package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DoNothing extends CommandBase {

    DriveSubsystem _drive;
    IntakeSubsystem _intake;
    ShooterSubsystem _shooter;

    public DoNothing(DriveSubsystem drive, IntakeSubsystem intake, ShooterSubsystem shooter) {
        _drive = drive;
        _intake = intake;
        _shooter = shooter;
    }

    @Override
    public void initialize() {
        _drive.tankDriveVolts(0.0, 0.0);
        _intake.stop(true);
        _shooter.update(0.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
