package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootLow extends CommandBase {

    private final ShooterSubsystem _shooter;
    private final IntakeSubsystem _intake;

    private int _initialBallCount;

    public ShootLow(ShooterSubsystem shooter, IntakeSubsystem intake) {
        _shooter = shooter;
        _intake = intake;
        addRequirements(
            _shooter,
            _intake
        );
    }

    @Override
    public void initialize() {
        _shooter.update(-0.7);
        _initialBallCount = _intake.getBallCount();
    }

    @Override
    public void execute() {
        if (_shooter.isAtSpeed()) {
            _intake.feed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        _shooter.update(0.0);
    }

    @Override
    public boolean isFinished() {
        return (_intake.getBallCount() < _initialBallCount) || (_intake.getBallCount() == 0);
    }
}
