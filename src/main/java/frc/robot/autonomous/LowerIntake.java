package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class LowerIntake extends CommandBase {

    private IntakeSubsystem _intake;

    public LowerIntake(IntakeSubsystem intake) {
        _intake = intake;
    }
    
    @Override
    public void initialize() {
        _intake.lowerIntake();
        _intake.intake();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
