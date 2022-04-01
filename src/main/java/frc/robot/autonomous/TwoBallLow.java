package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ShootClose;
import frc.robot.commands.ShootFar;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TwoBallLow extends SequentialCommandGroup {

    private DriveSubsystem _drive;
    private IntakeSubsystem _intake;
    private ShooterSubsystem _shooter;

    public TwoBallLow(
        DriveSubsystem drive, 
        IntakeSubsystem intake, 
        ShooterSubsystem shooter,
        Trajectory trajectory,
        Trajectory secondTrajectory
    ) {
        _drive = drive;
        _intake = intake;
        _shooter = shooter;

        RamseteCommand ramseteCommand =
          new RamseteCommand(
            trajectory,
            _drive::getPose,
            new RamseteController(Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
              Constants.DriveConstants.ksVolts,
              Constants.DriveConstants.kvVoltSecondsPerMeter,
              Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
              Constants.DriveConstants.kDriveKinematics,
            _drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),          
            _drive::tankDriveVolts, // RamseteCommand passes volts to the callback
            _drive
          );

        RamseteCommand secondRamseteCommand =
          new RamseteCommand(
            secondTrajectory,
            _drive::getPose,
            new RamseteController(Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
              Constants.DriveConstants.ksVolts,
              Constants.DriveConstants.kvVoltSecondsPerMeter,
              Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
              Constants.DriveConstants.kDriveKinematics,
            _drive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),          
            _drive::tankDriveVolts, // RamseteCommand passes volts to the callback
            _drive
          );


        addCommands(
          new InstantCommand(() -> _drive.resetOdometry(trajectory.getInitialPose()), _drive),
          new ShootClose(_shooter, _intake),
          ramseteCommand,
          new InstantCommand(() -> _intake.stop(false), _intake)
        );
    }
}
