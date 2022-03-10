// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeStates;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.DefaultIntake;
import frc.robot.commands.ShootLow;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem _driveSubsystem = new DriveSubsystem();
  private final IntakeSubsystem _intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem _shooterSubsystem = new ShooterSubsystem();

  private final RobotInput _robotInput = new RobotInput(
    new PS4Controller(Constants.Controller.PORT_PS4),
    new GenericHID(Constants.Controller.PORT_EXTREME));

  // A chooser for autonomous commands
  private final SendableChooser<Command> _autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    _driveSubsystem.setDefaultCommand(new DefaultDrive(_driveSubsystem, _robotInput));
    _intakeSubsystem.setDefaultCommand(new DefaultIntake(_intakeSubsystem));

    SmartDashboard.putData(_intakeSubsystem);
    SmartDashboard.putData(_shooterSubsystem);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses
   */
  private void configureButtonBindings() {
    final var intakeButton = _robotInput.getIntakeButton();
    final var ejectButton = _robotInput.getEjectButton();
    final var ejectLowerButton = _robotInput.getEjectLowerButton();
    final var shootLowButton = _robotInput.getShootLowButton();

    intakeButton.whenPressed(() -> _intakeSubsystem.setState(IntakeStates.Intake));
    ejectButton.whenPressed(() -> _intakeSubsystem.setState(IntakeStates.Eject));
    ejectLowerButton.whenPressed(() -> _intakeSubsystem.setState(IntakeStates.EjectLower));
    shootLowButton.whenPressed(new ShootLow(_shooterSubsystem, _intakeSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return _autoChooser.getSelected();
  }

  public int getBallCount() {
    return _intakeSubsystem.getBallCount();
  }
}
