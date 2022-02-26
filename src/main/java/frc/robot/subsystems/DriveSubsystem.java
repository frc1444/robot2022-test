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

        for (TalonFX talonFX : new TalonFX[] { leftDrive, rightDrive , leftFollower, rightFollower}) {
            talonFX.configFactoryDefault();
            talonFX.setNeutralMode(NeutralMode.Brake);

            // So we tuned our swerve drive in like 2018, then moved from CIM to Falcons, so this just lets us
            //   use the PID we tuned using the CIMs
            double ratio = Constants.CIMCODER_COUNTS_PER_REVOLUTION / Constants.FALCON_ENCODER_COUNTS_PER_REVOLUTION;
            talonFX.config_kP(Constants.SLOT_INDEX, 1.5 * ratio);
            talonFX.config_kF(Constants.SLOT_INDEX, 1.0 * ratio);
            talonFX.configClosedloopRamp(.40);
        }

        rightDrive.setInverted(TalonFXInvertType.Clockwise);

        // We could consider using WPI_TalonFX along with the DifferentialDrive class later,
        //   but keep in mind we really want to make sure that we use the velocity control. By default
        //   DifferentialDrive does not use velocity control
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
