package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotInput;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem _driveSubsystem;
    private final RobotInput _robotInput;

    private final PIDController turnController;

    public DriveCommand(DriveSubsystem driveSubsystem, RobotInput robotInput) {
        addRequirements(driveSubsystem);
        _driveSubsystem = driveSubsystem;
        _robotInput = robotInput;
        turnController = TurnToAngle.createPIDController();
    }

    public static Rotation2d getHeadingTowardsGoal(Pose2d robotPose) {
        Translation2d translation = Constants.FieldConstants.GOAL_CENTER.minus(robotPose.getTranslation());
        return new Rotation2d(Math.atan2(translation.getY(), translation.getX()));
    }

    @Override
    public void initialize() {
        turnController.reset();
    }

    @Override
    public void execute() {
        final double steer;
        if (_robotInput.isTurnTowardsGoalDown()) {
            Pose2d pose = _driveSubsystem.getPoseWithVision();
            Rotation2d desiredHeading = getHeadingTowardsGoal(pose);
            Rotation2d currentHeading = pose.getRotation();
            // We don't care if steer isn't in the range -1 to 1. curvatureDrive will clamp as needed
            steer = turnController.calculate(currentHeading.getDegrees(), desiredHeading.getDegrees());
        } else {
            turnController.reset();
            steer = _robotInput.getSteer();
        }
        _driveSubsystem.curvatureDrive(_robotInput.getForward(), steer);
    }
}
