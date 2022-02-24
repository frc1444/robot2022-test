package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends CommandBase {
    private final DriveSubsystem _drive;
    private final DoubleSupplier _forward;
    private final DoubleSupplier _rotation;

      /**
   * Creates a new DefaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param rotation The control input for turning
   */
  public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
    _drive = subsystem;
    _forward = forward;
    _rotation = rotation;
    addRequirements(_drive);
  }

  @Override
  public void execute() {
    _drive.arcadeDrive(_forward.getAsDouble(), _rotation.getAsDouble());
  }
}

