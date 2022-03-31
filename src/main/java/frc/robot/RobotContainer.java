// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.autonomous.AutonomousCommandBuilder;
import frc.robot.autonomous.DriveDistance;
import frc.robot.commands.DoNothing;
import frc.robot.commands.QuickLeft;
import frc.robot.commands.QuickRight;
import frc.robot.commands.ShootFar;
import frc.robot.commands.ShootClose;
import frc.robot.commands.Stop;
import frc.robot.util.CommandUtil;

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
  private final DriveSubsystem _drive;
  private final IntakeSubsystem _intake;
  private final ShooterSubsystem _shooter;
  private final ClimbSubsystem _climb;

  private final RobotInput _robotInput = new RobotInput(
    new PS4Controller(Constants.Controller.PORT_PS4_DRIVER),
    new PS4Controller(Constants.Controller.PORT_PS4_OPERATOR), new GenericHID(Constants.Controller.PORT_DRIVER_RUMBLE));

  // A chooser for autonomous commands
  private final SendableChooser<Command> _autoChooser = new SendableChooser<>();

  private final HashMap<String, Command> _autoCommands = new HashMap<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(HashMap<String, Trajectory> trajectories) {

    _pneumaticsHub = new PneumaticHub(Constants.CanIds.PNEUMATIC_HUB);

    _drive = new DriveSubsystem(
      _pneumaticsHub.makeDoubleSolenoid(
        Constants.PneumaticPortIds.SHIFTER_FWD, 
        Constants.PneumaticPortIds.SHIFTER_REV)
    );

    _intake = new IntakeSubsystem(
      _pneumaticsHub.makeDoubleSolenoid(
        Constants.PneumaticPortIds.INTAKE_FWD,
        Constants.PneumaticPortIds.INTAKE_REV
      )
    );

    _shooter = new ShooterSubsystem(
      _pneumaticsHub.makeDoubleSolenoid(
        Constants.PneumaticPortIds.SHOOTER_HOOD_FWD,
        Constants.PneumaticPortIds.SHOOTER_HOOD_REV
      )
    );
    _climb = new ClimbSubsystem(
        _pneumaticsHub.makeDoubleSolenoid(Constants.PneumaticPortIds.CLIMB_FWD, Constants.PneumaticPortIds.CLIMB_REV)
    );
    
    _drive.setDefaultCommand(new DriveCommand(_drive, _robotInput));

    // _intake.setDefaultCommand(
      // new RunCommand(() -> _intake.update()
      // , _intake));

    _climb.setDefaultCommand(new RunCommand(() -> {
        _climb.update(_robotInput.getClimbStage1(), _robotInput.getClimbStage2());
    }, _climb));

    // Configure the button bindings
    configureButtonBindings();

    // Pair up saved trajectories with appropriate auto commands
    buildAutoCommands(trajectories);

    // Put the auto commands to a drop down in the SmartDashboard
    SmartDashboard.putData("Auto Mode",_autoChooser);

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
    final var shooterSpinUpTrigger = _robotInput.getShooterSpinUp();
    final var shooterSpinDownTrigger = _robotInput.getShooterSpinDown();

    final var shiftHighTrigger = _robotInput.getShiftHigh();
    final var shiftLowTrigger = _robotInput.getShiftLow();
    final var quickLeftTrigger = _robotInput.getQuickLeft();
    final var quickRightTrigger = _robotInput.getQuickRight();

    final var visionOn = _robotInput.getVisionOn();
    final var visionOff = _robotInput.getVisionOff();

    final var releaseClimbHook = _robotInput.getReleaseClimbHook();

    intakeTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth).whenActive(() -> _intake.intake());
    ejectTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth).whenActive(() -> _intake.eject(true));
    ejectLowerTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth).whenActive(() -> _intake.eject(false));
    shootLowTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth)
      .whileActiveContinuous(new ShootClose(_shooter, _intake));
    shootHighTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth)
      .whileActiveContinuous(new ShootFar(_shooter, _intake));
    raiseIntakeTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth).whenActive(() -> _intake.raiseIntake());
    lowerIntakeTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth).whenActive(() -> _intake.lowerIntake());
    stopTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth).whenActive(new Stop(_shooter, _intake));
    /*
    shooterSpinUpTrigger.debounce(Constants.INPUT_DEBOUNCE).whenActive(
      () -> _shooter.update(Constants.ShooterConstants.SHOOT_FAR_SPEED)
    );
    shooterSpinDownTrigger.debounce(Constants.INPUT_DEBOUNCE).whenActive(
      () -> _shooter.update(0.0)
    );
    */

    shiftHighTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth).whenActive(() -> _drive.shiftHigh());
    shiftLowTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth).whenActive(() -> _drive.shiftLow());
    quickLeftTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth)
      .whileActiveContinuous(new QuickLeft(_drive).withTimeout(5));
    quickRightTrigger.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth)
      .whileActiveContinuous(new QuickRight(_drive).withTimeout(5));
    visionOn.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth).whenActive(CommandUtil.runWhenDisabled(new InstantCommand(() -> _drive.getVisionState().setEnabled(true))));
    visionOff.debounce(Constants.INPUT_DEBOUNCE, DebounceType.kBoth).whenActive(CommandUtil.runWhenDisabled(new InstantCommand(() -> _drive.getVisionState().setEnabled(false))));
    releaseClimbHook.whenActive(() -> _climb.releaseHook()).whenInactive(() -> _climb.engageHook());
    _robotInput.getDriverControllerIndicate()
        .whenActive(CommandUtil.runWhenDisabled(new InstantCommand(() -> _robotInput.getDriveRumble().setRumble(GenericHID.RumbleType.kLeftRumble, 0.5))))
        .whenInactive(CommandUtil.runWhenDisabled(new InstantCommand(() -> _robotInput.getDriveRumble().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0))));
  }

  private void buildAutoCommands(HashMap<String, Trajectory> trajectories) {
    if (trajectories == null || trajectories.size() <= 0) {
      return;
    }

    var keySet = trajectories.keySet();

    _autoChooser.setDefaultOption("do nothing", new DoNothing(_drive, _intake, _shooter));
    _autoChooser.addOption("leave tarmac", new DriveDistance(_drive, 2));

    for (var key : keySet) {
      _autoCommands.put(key,
        AutonomousCommandBuilder.buildAutoCommand(
          key, 
          trajectories.get(key), 
          _drive, _shooter, _intake)
      );
        
      _autoChooser.addOption(key, _autoCommands.get(key));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return _autoChooser.getSelected();
  }

  public double getShooterSetpoint() {
    return _shooter.getSetpoint();
  }

  public double getShooterRpm() {
    return _shooter.getCurrentRpm();
  }

  public double getDriveSetpoint() {
    return _drive.getSetpoint();
  }

  public double getAngle() {
    return _drive.getHeading();
  }

  public void updateShooterPid(double kP, double kI, double kD) {
    _shooter.updatePid(kP, kI, kD);
  }

  public Pose2d getPose() {
    return _drive.getPose();
  }
  
}
