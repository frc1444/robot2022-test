package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.util.FalconVelocityConverter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX _shooter;

    public ShooterSubsystem() {
        _shooter = new TalonFX(Constants.CanIds.SHOOTER);
        _shooter.configFactoryDefault();
        _shooter.setNeutralMode(NeutralMode.Coast);
        _shooter.config_kP(Constants.SLOT_INDEX, 0.06);
        _shooter.config_kF(Constants.SLOT_INDEX, 0.04);
        _shooter.configClosedloopRamp(.40);
        _shooter.configOpenloopRamp(.40);
    }

    public void update(double speed) {
        _shooter.set(TalonFXControlMode.Velocity, FalconVelocityConverter.percentToVelocity(speed));
    }

    public boolean isAtSpeed() {
        return true;
    }
}
