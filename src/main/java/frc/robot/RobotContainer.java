// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.ShootHigh;
import frc.robot.commands.ShootLow;
import frc.robot.commands.Stop;

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
  
  private final PneumaticHub _pneumaticsHub;
  private final DriveSubsystem _driveSubsystem;
  private final IntakeSubsystem _intakeSubsystem;
  private final ShooterSubsystem _shooterSubsystem = new ShooterSubsystem();

  private final RobotInput _robotInput = new RobotInput(
    new PS4Controller(Constants.Controller.PORT_PS4_DRIVER),
    new PS4Controller(Constants.Controller.PORT_PS4_OPERATOR));

  // A chooser for autonomous commands
  private final SendableChooser<Command> _autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

  _pneumaticsHub = new PneumaticHub(Constants.CanIds.PNEUMATIC_HUB);

  _driveSubsystem = new DriveSubsystem(
    _pneumaticsHub.makeDoubleSolenoid(
      Constants.PneumaticPortIds.SHIFTER_FWD, 
      Constants.PneumaticPortIds.SHIFTER_REV)
  );

  _intakeSubsystem = new IntakeSubsystem(
    _pneumaticsHub.makeDoubleSolenoid(
      Constants.PneumaticPortIds.INTAKE_FWD,
      Constants.PneumaticPortIds.INTAKE_REV
    )
  );
  
    _driveSubsystem.setDefaultCommand(
      new RunCommand(() -> _driveSubsystem.arcadeDrive(_robotInput.getForward(), _robotInput.getSteer())
      , _driveSubsystem));

    _intakeSubsystem.setDefaultCommand(
      new RunCommand(() -> _intakeSubsystem.update()
      , _intakeSubsystem));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses
   */
  private void configureButtonBindings() {

    final var intakeTrigger = _robotInput.getIntakeButton();
    final var ejectTrigger = _robotInput.getEjectButton();
    final var ejectLowerTrigger = _robotInput.getEjectLowerButton();
    final var shootLowTrigger = _robotInput.getShootLowButton();
    final var shootHighTrigger = _robotInput.getShootHighButton();
    final var raiseIntakeTrigger = _robotInput.getRaiseIntake();
    final var lowerIntakeTrigger = _robotInput.getLowerIntake();
    final var stopTrigger = _robotInput.getStopButton();
    final var shiftHighTrigger = _robotInput.getShiftHigh();
    final var shiftLowTrigger = _robotInput.getShiftLow();

    intakeTrigger.whenActive(() -> _intakeSubsystem.intake());
    ejectTrigger.whenActive(() -> _intakeSubsystem.eject(true));
    ejectLowerTrigger.whenActive(() -> _intakeSubsystem.eject(false));
    shootLowTrigger.whileActiveContinuous(new ShootLow(_shooterSubsystem, _intakeSubsystem));
    shootHighTrigger.whileActiveContinuous(new ShootHigh(_shooterSubsystem, _intakeSubsystem));
    raiseIntakeTrigger.whenActive(() -> _intakeSubsystem.raiseIntake());
    lowerIntakeTrigger.whenActive(() -> _intakeSubsystem.lowerIntake());
    stopTrigger.whenActive(new Stop(_shooterSubsystem, _intakeSubsystem));
    shiftHighTrigger.whenActive(() -> _driveSubsystem.shiftHigh());
    shiftLowTrigger.whenActive(() -> _driveSubsystem.shiftLow());
    
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

  public double getShooterSetpoint() {
    return _shooterSubsystem.getSetpoint();
  }

  public double getShooterRpm() {
    return _shooterSubsystem.getCurrentRpm();
  }

  public double getDriveSetpoint() {
    return _driveSubsystem.getSetpoint();
  }

  public double getDriveVelocity() {
    return _driveSubsystem.getVelocity();
  }

  public double getAngle() {
    return _driveSubsystem.getAngle();
  }

  public void updateShooterPid(double kP, double kI, double kD) {
    _shooterSubsystem.updatePid(kP, kI, kD);
  }
  
}
