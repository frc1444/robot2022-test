package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.FalconVelocityConverter;

public class DriveSubsystem extends SubsystemBase {

    private final TalonFX _leftDrive;
    private final TalonFX _rightDrive;
    private final DoubleSolenoid _shiftSolenoid;
    private final Pigeon2 _gyro;

    private final DifferentialDriveOdometry _odomemtry;

    public DriveSubsystem(DoubleSolenoid shiftSolenoid) {
        _leftDrive = new TalonFX(Constants.CanIds.LEFT_DRIVE_LEADER);
        _rightDrive = new TalonFX(Constants.CanIds.RIGHT_DRIVE_LEADER);
        _shiftSolenoid = shiftSolenoid;

        TalonFX leftFollower = new TalonFX(Constants.CanIds.LEFT_DRIVE_FOLLOWER);
        leftFollower.follow(_leftDrive, FollowerType.PercentOutput);
        leftFollower.setInverted(TalonFXInvertType.FollowMaster); // same direction as master

        TalonFX rightFollower = new TalonFX(Constants.CanIds.RIGHT_DRIVE_FOLLOWER);
        rightFollower.follow(_rightDrive, FollowerType.PercentOutput);
        rightFollower.setInverted(TalonFXInvertType.FollowMaster); // same direction as master

        for (TalonFX talonFX : new TalonFX[] { _leftDrive, _rightDrive, leftFollower, rightFollower}) {
            talonFX.configFactoryDefault();
            talonFX.setNeutralMode(NeutralMode.Coast);

            talonFX.config_kP(Constants.SLOT_INDEX, Constants.DriveConstants.DRIVE_KP);
            talonFX.config_kF(Constants.SLOT_INDEX, Constants.DriveConstants.DRIVE_KF);
            talonFX.configClosedloopRamp(DriveConstants.LOW_GEAR_RAMP_RATE);
            talonFX.configNeutralDeadband(DriveConstants.DEAD_ZONE);
            talonFX.configVoltageCompSaturation(DriveConstants.VOLTS_FOR_COMPENSATION);
            talonFX.enableVoltageCompensation(true);
        }

        _rightDrive.setInverted(TalonFXInvertType.Clockwise);

        // Shift low to initialize state (needed for getting proper gear ratios)
        shiftLow();

        _gyro = new Pigeon2(Constants.CanIds.PIGEON_IMU);
        _gyro.configFactoryDefault();

        resetEncoders();
        _odomemtry = new DifferentialDriveOdometry(new Rotation2d(getHeadingRadians()));
    }

    @Override
    public void periodic() {
        _odomemtry.update(
            new Rotation2d(getHeadingRadians()), 
            getLeftSensorPosition(), 
            getRightSensorPosition()
        );
    }

    /**
     * Drives the robot using arcade controls.
       *
       * @param fwd the commanded forward movement
       * @param rot the commanded rotation
       */
    public void curvatureDrive(double fwd, double rot) {

         // We can easily change this to arcade drive if we feel like it
         DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(fwd, rot, true);

         //if (speeds.left == 0.0) {
         //    _leftDrive.neutralOutput();
         //} else {
             _leftDrive.set(TalonFXControlMode.Velocity, FalconVelocityConverter.percentToVelocity(speeds.left));
         //}
         //if (speeds.right == 0.0) {
           //  _rightDrive.neutralOutput();
         //} else {
             _rightDrive.set(TalonFXControlMode.Velocity, FalconVelocityConverter.percentToVelocity(speeds.right));
         //}
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        _leftDrive.set(TalonFXControlMode.PercentOutput, leftVolts / DriveConstants.VOLTS_FOR_COMPENSATION);
        _rightDrive.set(TalonFXControlMode.PercentOutput, rightVolts / DriveConstants.VOLTS_FOR_COMPENSATION);
    }


    public void shiftLow() {
        _shiftSolenoid.set(DriveConstants.SHIFT_LOW);
    }

    public void shiftHigh() {
        _shiftSolenoid.set(DriveConstants.SHIFT_HIGH);
    }

    public Pose2d getPose() {
        return _odomemtry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            getLeftSensorVelocity(), 
            getRightSensorVelocity());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        _odomemtry.resetPosition(pose, new Rotation2d(getHeadingRadians()));
    }

    public double getSetpoint() {
        return (_leftDrive.getClosedLoopTarget() + _rightDrive.getClosedLoopTarget());
    }

    public double getHeading() {
        return Math.IEEEremainder(_gyro.getYaw(), 360);
    }

    public double getHeadingRadians() {
        return Math.toRadians(getHeading());
    }

    public double getAverageEncoderDistance() {
        return (getLeftSensorPosition() + getRightSensorPosition()) / 2.0;
    }

    public void zeroHeading() {
        _gyro.setYaw(0.0);
    }

    public double getTurnRate() {
        double[] xyz = new double[3];
        _gyro.getRawGyro(xyz);
        return -xyz[2];
    }

    private void resetEncoders() {
        _leftDrive.setSelectedSensorPosition(0);
        _rightDrive.setSelectedSensorPosition(0);
    }

    private double getLeftSensorPosition() {
        return rawTalonPositionToMeters(_leftDrive.getSelectedSensorPosition());
    }

    private double getRightSensorPosition() {
        return rawTalonPositionToMeters(_rightDrive.getSelectedSensorPosition());
    }

    private double getLeftSensorVelocity() {
        return rawTalonVelocityToMetersPerSecond(_leftDrive.getSelectedSensorVelocity());
    }

    private double getRightSensorVelocity() {
        return rawTalonVelocityToMetersPerSecond(_rightDrive.getSelectedSensorVelocity());
    }

    private double rawTalonPositionToMeters(double counts) {
        double metersPerTick = DriveConstants.WHEEL_CIRCUMFERENCE_METERS / Constants.FALCON_ENCODER_COUNTS_PER_REVOLUTION;
        return (metersPerTick * counts) / getCurrentGearing();
    }

    private double rawTalonVelocityToMetersPerSecond(double countsPer100ms) {
        double countsPerSecond = countsPer100ms * 10;
        double metersPerTick = DriveConstants.WHEEL_CIRCUMFERENCE_METERS / Constants.FALCON_ENCODER_COUNTS_PER_REVOLUTION;
        return (countsPerSecond * metersPerTick) / getCurrentGearing();
    }

    private double getCurrentGearing() {
        if (_shiftSolenoid.get() == DriveConstants.SHIFT_LOW) {
            return DriveConstants.LOW_GEARING;
        }
        else {
            return DriveConstants.HIGH_GEARING;
        }
    }
}
