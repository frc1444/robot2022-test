package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotInput;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {
    private final DriveSubsystem _drive;
    private final RobotInput _robotInput;
    // private final Pigeon2 _pigeonImu;

  /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command will run on.
   * @param robotInput The robotInput
   */
  public DefaultDrive(DriveSubsystem subsystem, RobotInput robotInput) {
    _drive = subsystem;
    _robotInput = robotInput; // We pass the entire robot input in because it allows for easy changes when we need to add more complex bindings
    addRequirements(_drive);
  }

  @Override
  public void execute() {
    _drive.arcadeDrive(_robotInput.getForward(), _robotInput.getSteer());
  }

  @Override
  public void end(boolean interrupted) {
    _drive.arcadeDrive(0.0, 0.0);
  }
}

