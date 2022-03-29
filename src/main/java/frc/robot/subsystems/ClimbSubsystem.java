package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {

    private final DoubleSolenoid hookReleaseSolenoid;
    private final TalonFX stage1;
    private final TalonFX stage2;

    public ClimbSubsystem(DoubleSolenoid hookReleaseSolenoid) {
        this.hookReleaseSolenoid = hookReleaseSolenoid;
        stage1 = new TalonFX(Constants.CanIds.FIRST_STAGE_CLIMB);
        stage2 = new TalonFX(Constants.CanIds.SECOND_STAGE_CLIMB);

        for (TalonFX controller : new TalonFX[] { stage1, stage2 }) {
            controller.configFactoryDefault();
            controller.setNeutralMode(NeutralMode.Brake);
        }
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            engageHook();
        }
    }

    /** Updates the speeds of the climb motors. A positive value makes the climb go up. Negative values are for bringing the robot up after hooking on. */
    public void update(double stage1Speed, double stage2Speed) {
        stage1.set(TalonFXControlMode.PercentOutput, adjust(stage1Speed, Constants.ClimbConstants.MAX_SPEED_STAGE_FIRST, Constants.ClimbConstants.MAX_SPEED_STAGE_FIRST_REVERSE));
        stage2.set(TalonFXControlMode.PercentOutput, adjust(stage2Speed, Constants.ClimbConstants.MAX_SPEED_STAGE_SECOND, Constants.ClimbConstants.MAX_SPEED_STAGE_SECOND_REVERSE));
    }
    private static double adjust(double speed, double maxForward, double maxReverse) {
        if (speed >= 0) {
            return speed * maxForward;
        }
        return speed * maxReverse;
    }

    public void releaseHook() {
        hookReleaseSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    public void engageHook() {
        hookReleaseSolenoid.set(DoubleSolenoid.Value.kForward);
    }
}
