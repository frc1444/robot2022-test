package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotInput;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OperatorCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final RobotInput robotInput;

    public OperatorCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, RobotInput robotInput) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.robotInput = robotInput;
    }

    @Override
    public void execute() {
        intakeSubsystem.update(robotInput.getIntakeSpeed(), robotInput.getManualIndexerSpeed());
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.update(0.0, 0.0);
    }
}
