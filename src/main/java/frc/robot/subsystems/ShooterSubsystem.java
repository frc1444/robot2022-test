package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.util.FalconVelocityConverter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX _shooter;
    private final TalonFX _hood;
    private double _shooterSetpoint;

    private final double HOOD_SETPOINT = 1.0;
    private final double SHOOTER_ERROR_LIMIT = 200;

    public ShooterSubsystem() {
        _shooter = new TalonFX(Constants.CanIds.SHOOTER);
        _hood = new TalonFX(Constants.CanIds.SHOOTER_HOOD);
        
        _shooter.configFactoryDefault();
        _shooter.setNeutralMode(NeutralMode.Coast);
        _shooter.config_kP(Constants.SLOT_INDEX, 0.12);
        _shooter.config_kF(Constants.SLOT_INDEX, 0.04);
        _shooter.config_kI(Constants.SLOT_INDEX, 0.001);
        //_shooter.configClosedloopRamp(.40);
        //_shooter.configOpenloopRamp(.40);
        _shooter.configPeakOutputForward(0.0);

        _hood.configFactoryDefault();
        _hood.setNeutralMode(NeutralMode.Coast);
        _shooter.configPeakOutputReverse(0.0);
        _shooter.config_kP(Constants.SLOT_INDEX, 0.06);
        _shooter.config_kF(Constants.SLOT_INDEX, 0.02);

        _shooterSetpoint = 0.0;
    }

    public void update(double speed) {
        if (speed == 0.0) {
            _shooter.neutralOutput();
            _hood.neutralOutput();
        }
        else {
            _shooterSetpoint = FalconVelocityConverter.percentToVelocity(speed);
            _shooter.set(TalonFXControlMode.Velocity, _shooterSetpoint);

            var hoodSetpoint = FalconVelocityConverter.percentToVelocity(HOOD_SETPOINT);
            _hood.set(TalonFXControlMode.Velocity, hoodSetpoint);
        }
    }

    public boolean isAtSetpoint() {
        var rpmError = Math.abs(_shooter.getSelectedSensorVelocity() - _shooterSetpoint);
        return rpmError < SHOOTER_ERROR_LIMIT;
    }

    public double getSetpoint() {
        return _shooterSetpoint;
    }

    public double getCurrentRpm() {
        return _shooter.getSelectedSensorVelocity();
    }

    public void updatePid(double kP, double kI, double kD) {
        _shooter.config_kP(Constants.SLOT_INDEX, kP);
        _shooter.config_kI(Constants.SLOT_INDEX, kI);
        _shooter.config_kD(Constants.SLOT_INDEX, kD);
    }
}
