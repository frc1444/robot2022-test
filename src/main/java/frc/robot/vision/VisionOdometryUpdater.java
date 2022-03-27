package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

import static java.lang.Math.abs;

public class VisionOdometryUpdater {
    private static final double VISION_DELAY_TIME_ALLOWED = .3;

    private final DriveSubsystem _driveSubsystem;
    private final DifferentialDriveOdometry _odometry;

    private double lastTimestamp = 0;
    private int similarCount = 0;
    private Transform2d lastSurroundingTransform = null;

    public VisionOdometryUpdater(DriveSubsystem driveSubsystem, DifferentialDriveOdometry odometry) {
        _driveSubsystem = driveSubsystem;
        _odometry = odometry;
    }

    public void updateNoVision() {
        similarCount = 0;
        lastSurroundingTransform = null;
    }

    public void update(Surrounding surrounding, boolean updateOdometry) {
        double timestamp = surrounding.getTimestamp();
        final double lastTimestamp = this.lastTimestamp;
        this.lastTimestamp = timestamp;

        if (timestamp + VISION_DELAY_TIME_ALLOWED < timestamp) {
            setVisionStatus("Too Old");
            return; // this is too old!
        }
        if (timestamp <= lastTimestamp) {
            return; // this isn't new!
        }
        Transform2d transform = surrounding.getTransform();
        Transform2d lastSurroundingTransform = this.lastSurroundingTransform;
        this.lastSurroundingTransform = transform;
        if(lastSurroundingTransform == null){
            return;
        }
        if(lastSurroundingTransform.getTranslation().getDistance(transform.getTranslation()) < .2 && abs(lastSurroundingTransform.getRotation().minus(transform.getRotation()).getDegrees()) < 7.0){
            similarCount++;
        } else {
            similarCount = 0;
        }
        if(similarCount < 3){
            setVisionStatus("Jumpy Vision Data");
            return;
        }
        if (!updateOdometry) {
            setVisionStatus("Vision data, but not updating");
            return;
        }
        Translation2d visionRelativePosition = transform.getTranslation();
        // note that this year we will not use transform.getRotation() because that should always be 0 degrees this year
        Rotation2d robotHeading = new Rotation2d(_driveSubsystem.getHeadingRadians());
        Translation2d newPosition = Constants.GOAL_CENTER.plus(visionRelativePosition.unaryMinus().rotateBy(robotHeading));
        // TODO I don't really know if we want the resetPosition method to update the gyro offset. This shouldn't be done during autonomous, so it probably don't matter
        _odometry.resetPosition(new Pose2d(newPosition, robotHeading), robotHeading);
        setVisionStatus("Good vision");
    }
    private void setVisionStatus(String statusMessage){
        // do nothing now, could use this later and put something in Shuffleboard
        SmartDashboard.putString("Vision status", statusMessage);
    }
}
