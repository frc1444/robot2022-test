package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntake extends CommandBase {
    private final IntakeSubsystem _intake;

    public DefaultIntake (IntakeSubsystem intake) {
        _intake = intake;
        addRequirements(_intake);
    }

    @Override
    public void execute() {
        _intake.update();
    }
  
    @Override
    public void end(boolean interrupted) {
        _intake.stop();
    }
}
