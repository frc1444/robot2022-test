package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootLow extends CommandBase {

    private final ShooterSubsystem _shooter;
    private final IntakeSubsystem _intake;

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
    }

    @Override
    public void execute() {
        if (_shooter.isAtSetpoint()) {
            _intake.feed();
        }
    }

    @Override
    public void end(boolean interrupted) {
        _shooter.update(0.0);
        _intake.stop(false);
    }

    @Override
    public boolean isFinished() {
        return _intake.wasBallFed();
    }
}
