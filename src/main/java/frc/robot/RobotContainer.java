// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.OperatorCommand;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem _driveSubsystem = new DriveSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final RobotInput robotInput = new RobotInput(
            new PS4Controller(Constants.Controller.PORT_PS4),
            new GenericHID(Constants.Controller.PORT_EXTREME)
    );

    // A chooser for autonomous commands
    private final SendableChooser<Command> _autoChooser = new SendableChooser<>();

    private final Command teleopCommand;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        Command driveCommand = new DefaultDrive(_driveSubsystem, robotInput);
        Command operatorCommand = new OperatorCommand(intakeSubsystem, shooterSubsystem, robotInput);
        teleopCommand = new ParallelCommandGroup(driveCommand, operatorCommand);
    }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return _autoChooser.getSelected();
  }
  public Command getTeleopCommand() {
      return teleopCommand;
  }
}
