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
import frc.robot.commands.TimedDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class OneBallLow extends SequentialCommandGroup {

    private DriveSubsystem _drive;
    private IntakeSubsystem _intake;
    private ShooterSubsystem _shooter;

    public OneBallLow(
        DriveSubsystem drive, 
        IntakeSubsystem intake, 
        ShooterSubsystem shooter,
        Trajectory trajectory
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

        addCommands(
          new InstantCommand(() -> _drive.resetOdometry(trajectory.getInitialPose()), _drive),
          new ShootClose(_shooter, _intake).withTimeout(3.0),
          new TimedDrive(drive).withTimeout(2.5),
          new InstantCommand(() -> _intake.stop(false), _intake),
          new InstantCommand(() -> _drive.tankDriveVolts(0.0, 0.0), _drive)
        );
    }
}
