package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import org.jetbrains.annotations.Nullable;

public class VisionOdometryUpdater {
    private static final double VISION_DELAY_TIME_ALLOWED = .3;

    private final DriveSubsystem _driveSubsystem;

    private double lastTimestamp = 0;
    private int similarCount = 0;
    private Translation2d lastRelativeGoalCenter = null;

    public VisionOdometryUpdater(DriveSubsystem driveSubsystem) {
        _driveSubsystem = driveSubsystem;
    }

    private static @Nullable Translation2d getRelativeGoalCenter(VisionInstant visionInstant) {
        int count = 0;
        double xSum = 0.0;
        double ySum = 0.0;
        for (Surrounding surrounding : visionInstant.getSurroundings()) {
            Transform2d transform2d = surrounding.getTransform();
            Translation2d goalCenter = transform2d.getTranslation().plus(new Translation2d(Constants.FieldConstants.UPPER_HUB_RADIUS_METERS, 0.0).rotateBy(transform2d.getRotation()));

//            Rotation2d surroundingTheta = new Rotation2d(Math.atan2(transform2d.getY(), transform2d.getX()));
//            if (Math.abs(surroundingTheta.minus(transform2d.getRotation()).getDegrees()) > 15.0) {
//                continue;
//            }


            // If we decide in the future to not include a particular surrounding, we would throw these 3 lines in an if statement
            xSum += goalCenter.getX();
            ySum += goalCenter.getY();
            count++;
        }
        SmartDashboard.putString("Using", count + "/" + visionInstant.getSurroundings().size());
        if (count == 0) {
            return null;
        }

        return new Translation2d(xSum / count, ySum / count);
    }

    private void resetSimilar() {
        similarCount = 0;
        lastRelativeGoalCenter = null;
    }

    public @Nullable Translation2d updateAndGetNewPosition(@Nullable VisionInstant visionInstant) {
        if (visionInstant == null) {
            resetSimilar();
            setVisionStatus("No vision connection");
            return null;
        }
        double timestamp = visionInstant.getTimestamp();
        final double lastTimestamp = this.lastTimestamp;
        this.lastTimestamp = timestamp;

        if (timestamp + VISION_DELAY_TIME_ALLOWED < Timer.getFPGATimestamp()) {
            resetSimilar();
            setVisionStatus("Too Old");
            return null;
        }
        if (timestamp <= lastTimestamp) {
            return null;
        }
        if (visionInstant.getSurroundings().isEmpty()) {
            resetSimilar();
            setVisionStatus("Connected, no surroundings");
            return null;
        }
        Translation2d relativeGoalCenter = getRelativeGoalCenter(visionInstant);
        if (relativeGoalCenter == null) {
            resetSimilar();
            setVisionStatus("Ignoring provided data");
            return null;
        }
        SmartDashboard.putNumber("goalX", relativeGoalCenter.getX());
        SmartDashboard.putNumber("goalY", relativeGoalCenter.getY());
        Translation2d lastRelativeGoalCenter = this.lastRelativeGoalCenter;
        this.lastRelativeGoalCenter = relativeGoalCenter;
        if(lastRelativeGoalCenter == null){
            return null;
        }
        if(lastRelativeGoalCenter.getDistance(relativeGoalCenter) < .2){
            similarCount++;
        } else {
            similarCount = 0;
        }
        if(similarCount < 3){
            setVisionStatus("Jumpy Vision Data");
            return null;
        }
        // note that this year we will not use transform.getRotation() because that should always be 0 degrees this year
        Rotation2d robotHeading = new Rotation2d(_driveSubsystem.getHeadingRadians());
        Translation2d newPosition = Constants.FieldConstants.GOAL_CENTER.plus(relativeGoalCenter.unaryMinus().rotateBy(robotHeading));
//        _odometry.resetPosition(new Pose2d(newPosition, robotHeading), robotHeading);
        setVisionStatus("Good vision. Similar: " + similarCount);
        SmartDashboard.putNumber("new position x", newPosition.getX());
        SmartDashboard.putNumber("new position y", newPosition.getY());
        return newPosition;
    }
    private void setVisionStatus(String statusMessage){
        // do nothing now, could use this later and put something in Shuffleboard
        SmartDashboard.putString("Vision status", statusMessage);
    }
}
