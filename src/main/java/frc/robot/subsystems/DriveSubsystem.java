package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    private final TalonFX leftDrive;
    private final TalonFX rightDrive;

    public DriveSubsystem() {
        leftDrive = new TalonFX(Constants.CanIds.LEFT_DRIVE_LEADER);
        rightDrive = new TalonFX(Constants.CanIds.RIGHT_DRIVE_LEADER);

        TalonFX leftFollower = new TalonFX(Constants.CanIds.LEFT_DRIVE_FOLLOWER);
        leftFollower.follow(leftDrive, FollowerType.PercentOutput);
        leftFollower.setInverted(TalonFXInvertType.FollowMaster); // same direction as master

        TalonFX rightFollower = new TalonFX(Constants.CanIds.RIGHT_DRIVE_FOLLOWER);
        rightFollower.follow(rightDrive, FollowerType.PercentOutput);
        rightFollower.setInverted(TalonFXInvertType.FollowMaster); // same direction as master

        for (TalonFX talonFX : new TalonFX[] { leftDrive, rightDrive, leftFollower, rightFollower}) {
            talonFX.configFactoryDefault();
            talonFX.setNeutralMode(NeutralMode.Brake);

            talonFX.config_kP(Constants.SLOT_INDEX, 0.06);
            talonFX.config_kF(Constants.SLOT_INDEX, 0.04);
            talonFX.configClosedloopRamp(.40);
            talonFX.configOpenloopRamp(.40);
        }

        rightDrive.setInverted(TalonFXInvertType.Clockwise);
    }

    private double rpmToVelocity(double rpm) {
        return rpm * Constants.FALCON_ENCODER_COUNTS_PER_REVOLUTION / Constants.CTRE_UNIT_CONVERSION; // maybe we might want to make falcons faster (they can go faster)
    }
    private double percentToVelocity(double percent) {
        // TODO We probably will never get to MAX_FALCON RPM, so we could consider lowering that value
        return rpmToVelocity(percent * Constants.MAX_FALCON_RPM);
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

         if (speeds.left == 0.0) {
             leftDrive.neutralOutput();
         } else {
             leftDrive.set(TalonFXControlMode.Velocity, percentToVelocity(speeds.left));
         }
         if (speeds.right == 0.0) {
             rightDrive.neutralOutput();
         } else {
             rightDrive.set(TalonFXControlMode.Velocity, percentToVelocity(speeds.right));
         }

    }
}
