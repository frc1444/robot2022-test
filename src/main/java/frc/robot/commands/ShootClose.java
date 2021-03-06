package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootClose extends CommandBase {

    private final ShooterSubsystem _shooter;
    private final IntakeSubsystem _intake;

    public ShootClose(ShooterSubsystem shooter, IntakeSubsystem intake) {
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
        _shooter.update(ShooterConstants.SHOOT_CLOSE_SPEED);
        _intake.clearBallFed();
    }

    @Override
    public void execute() {
        if (_shooter.isAtSetpointFor(0.1)) {
            _intake.feed();
        } else {
            _intake.setStopState(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        _shooter.update(0.0);
       // _intake.stop(false);
    }

    @Override
    public boolean isFinished() {
        return _intake.wasBallFed();
    }
}
