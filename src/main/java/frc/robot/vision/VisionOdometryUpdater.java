package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    private static Translation2d getRelativeGoalCenter(VisionInstant visionInstant) {
        if (visionInstant.getSurroundings().isEmpty()) {
            throw new IllegalArgumentException("You can only call getRelativeGoalCenter with a VisionInstant that has surroundings!");
        }
        int count = 0;
        double xSum = 0.0;
        double ySum = 0.0;
        for (Surrounding surrounding : visionInstant.getSurroundings()) {
            Transform2d transform2d = surrounding.getTransform();
            Translation2d goalCenter = transform2d.getTranslation().plus(new Translation2d(Constants.FieldConstants.UPPER_HUB_RADIUS_METERS, 0.0).rotateBy(transform2d.getRotation()));
            // If we decide in the future to not include a particular surrounding, we would throw these 3 lines in an if statement
            xSum += goalCenter.getX();
            ySum += goalCenter.getY();
            count++;
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

        if (timestamp + VISION_DELAY_TIME_ALLOWED < timestamp) {
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
        return newPosition;
    }
    private void setVisionStatus(String statusMessage){
        // do nothing now, could use this later and put something in Shuffleboard
        SmartDashboard.putString("Vision status", statusMessage);
    }
}
