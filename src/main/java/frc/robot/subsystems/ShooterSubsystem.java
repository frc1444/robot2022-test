package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.util.FalconVelocityConverter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.jetbrains.annotations.Nullable;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX _shooter;
    private final TalonFX _hood;
    private final DoubleSolenoid _hoodSolenoid;
    private double _shooterSetpoint;


    private @Nullable Double firstShooterAtSetpoint = null;


    public ShooterSubsystem(DoubleSolenoid hoodSolenoid) {
        _shooter = new TalonFX(Constants.CanIds.SHOOTER);
        _hood = new TalonFX(Constants.CanIds.SHOOTER_HOOD);
        _hoodSolenoid = hoodSolenoid;
        
        _shooter.configFactoryDefault();
        _shooter.setNeutralMode(NeutralMode.Coast);
        _shooter.config_kP(Constants.SLOT_INDEX, Constants.ShooterConstants.SHOOTER_KP);
        _shooter.config_kF(Constants.SLOT_INDEX, Constants.ShooterConstants.SHOOTER_KF);
        _shooter.config_kI(Constants.SLOT_INDEX, Constants.ShooterConstants.SHOOTER_KI);
        _shooter.config_kD(Constants.SLOT_INDEX, Constants.ShooterConstants.SHOOTER_KD);
        _shooter.configClosedloopRamp(Constants.ShooterConstants.SHOOTER_RAMP_RATE);
        _shooter.configOpenloopRamp(Constants.ShooterConstants.SHOOTER_RAMP_RATE);
        _shooter.configPeakOutputForward(0.0); // Prevent shooter from running the wrong direction

        _hood.configFactoryDefault();
        _hood.setNeutralMode(NeutralMode.Coast);
        _hood.configPeakOutputForward(0.0); // Prevent hood from running the wrong direction
        _hood.config_kP(Constants.SLOT_INDEX, Constants.ShooterConstants.HOOD_KP);
        _hood.config_kF(Constants.SLOT_INDEX, Constants.ShooterConstants.HOOD_KF);

        _shooterSetpoint = 0.0;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            update(0.0);
        }

        if (isAtSetpoint()) {
            double now = Timer.getFPGATimestamp();
            if (firstShooterAtSetpoint == null) {
                firstShooterAtSetpoint = now;
            }
        } else {
            firstShooterAtSetpoint = null;
        }
    }

    public void update(double speed) {
        if (speed == 0.0) {
            _shooter.neutralOutput();
            _hood.neutralOutput();
        }
        else {
            _shooterSetpoint = FalconVelocityConverter.percentToVelocity(speed);
            _shooter.set(TalonFXControlMode.Velocity, _shooterSetpoint);

            var hoodSetpoint = FalconVelocityConverter.percentToVelocity(
                Constants.ShooterConstants.HOOD_SETPOINT
            );
            _hood.set(TalonFXControlMode.Velocity, hoodSetpoint);
        }
    }

    public void setHoodLow() {
        _hoodSolenoid.set(Constants.ShooterConstants.HOOD_LOW);
    }

    public void setHoodHigh() {
        _hoodSolenoid.set(Constants.ShooterConstants.HOOD_HIGH);
    }

    public boolean isAtSetpoint() {
        var rpmError = Math.abs(_shooter.getSelectedSensorVelocity() - _shooterSetpoint);
        return rpmError < Constants.ShooterConstants.SHOOTER_ERROR_LIMIT;
    }
    public boolean isAtSetpointFor(double timeSeconds) {
        double now = Timer.getFPGATimestamp();
        Double firstShooterAtSetpoint = this.firstShooterAtSetpoint;
        if (firstShooterAtSetpoint == null) {
            return false;
        }
        double timeAtSetpoint = now - firstShooterAtSetpoint;
        return timeAtSetpoint >= timeSeconds;
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
