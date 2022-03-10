package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotInput;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OperatorCommand extends CommandBase {

    private final IntakeSubsystem _intake;
    private final ShooterSubsystem _shooter;
    private final RobotInput _robotInput;

    public OperatorCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, RobotInput robotInput) {
        _intake = intakeSubsystem;
        _shooter = shooterSubsystem;
        _robotInput = robotInput;

        addRequirements(
            _intake,
            _shooter
        );
    }

    @Override
    public void execute() {
        //_intake.update(_robotInput.getIntakeSpeed(), _robotInput.getManualIndexerSpeed());
    }

    @Override
    public void end(boolean interrupted) {
        _intake.stop();
    }
}