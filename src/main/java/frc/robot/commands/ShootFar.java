package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootFar extends CommandBase {

    private final ShooterSubsystem _shooter;
    private final IntakeSubsystem _intake;

//    private Double firstShooterAtSetpoint = null;

    public ShootFar(ShooterSubsystem shooter, IntakeSubsystem intake) {
        _shooter = shooter;
        _intake = intake;
        addRequirements(
            _shooter,
            _intake
        );
    }

    @Override
    public void initialize() {
        _shooter.setHoodLow();
        _shooter.update(ShooterConstants.SHOOT_FAR_SPEED);
        _intake.clearBallFed();
    }

    @Override
    public void execute() {
//        final boolean feed;
//        if (_shooter.isAtSetpoint()) {
//            double now = Timer.getFPGATimestamp();
//            if (firstShooterAtSetpoint == null) {
//                firstShooterAtSetpoint = now;
//            }
//            double timeAtSetpoint = now - firstShooterAtSetpoint;
//            feed = timeAtSetpoint
//            _intake.feed();
//        } else {
//            firstShooterAtSetpoint = null;
//        }
        if (_shooter.isAtSetpoint()) {
//            Timer.delay(0.1);
            _intake.feed();
        } else {
//            firstShooterAtSetpoint = null;
            _intake.stop(false);
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
