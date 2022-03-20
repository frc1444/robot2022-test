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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

            talonFX.config_kP(Constants.SLOT_INDEX, 0.12);
            talonFX.config_kF(Constants.SLOT_INDEX, 0.06);
            talonFX.configClosedloopRamp(.40);
            talonFX.configOpenloopRamp(.40);
            talonFX.configNeutralDeadband(0.04);    // in case we don't want to use the manual deadband in RobotInput
        }

        _rightDrive.setInverted(TalonFXInvertType.Clockwise);

        _gyro = new Pigeon2(Constants.CanIds.PIGEON_IMU);
        _gyro.configFactoryDefault();

        // TODO these values need to be set dynamically for different autonomous modes
        _odomemtry = new DifferentialDriveOdometry(new Rotation2d(_gyro.getYaw()), new Pose2d(0.0, 0.0, new Rotation2d()));
    }

    @Override
    public void periodic() {
        // update odometry      

        _odomemtry.update(
            new Rotation2d(getHeading()), 
            _leftDrive.getSelectedSensorPosition(), 
            _rightDrive.getSelectedSensorPosition()
        );
    }

    /**
     * Drives the robot using arcade controls.
       *
       * @param fwd the commanded forward movement
       * @param rot the commanded rotation
       */
     public void arcadeDrive(double fwd, double rot) {



         // We can easily change this to arcade drive if we feel like it
         DifferentialDrive.WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(fwd, rot, true);

         //if (speeds.left == 0.0) {
         //    _leftDrive.neutralOutput();
         //} else {
             _leftDrive.set(TalonFXControlMode.Velocity, FalconVelocityConverter.percentToVelocity(speeds.left));
         //}
         //if (speeds.right == 0.0) {
             _rightDrive.neutralOutput();
         //} else {
             _rightDrive.set(TalonFXControlMode.Velocity, FalconVelocityConverter.percentToVelocity(speeds.right));
         //}
    }

    public void shiftLow() {
        _shiftSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void shiftHigh() {
        _shiftSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public double getSetpoint() {
        return (_leftDrive.getClosedLoopTarget() + _rightDrive.getClosedLoopTarget());
    }

    public double getVelocity() {
        return (_leftDrive.getSelectedSensorVelocity() + _rightDrive.getSelectedSensorVelocity());
    }

    public double getHeading() {
        return Math.IEEEremainder(_gyro.getYaw(), 360);
    }
}
