package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootHigh extends CommandBase {

    private final ShooterSubsystem _shooter;
    private final IntakeSubsystem _intake;

    public ShootHigh(ShooterSubsystem shooter, IntakeSubsystem intake) {
        _shooter = shooter;
        _intake = intake;
        addRequirements(
            _shooter,
            _intake
        );
    }

    @Override
    public void initialize() {
        _shooter.setHoodHigh();
        _shooter.update(Constants.SHOOT_HIGH_SPEED);
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
