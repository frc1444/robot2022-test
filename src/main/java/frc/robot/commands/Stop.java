package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Stop extends CommandBase {

    private final ShooterSubsystem _shooter;
    private final IntakeSubsystem _intake;

    public Stop(ShooterSubsystem shooter, IntakeSubsystem intake) {
        _shooter = shooter;
        _intake = intake;
        addRequirements(
            _shooter,
            _intake
        );
    }

    @Override
    public void initialize() {
        _shooter.update(0.0);
        _intake.stop(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
