package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax _intake;
    private final CANSparkMax _singulateLeft;
    private final CANSparkMax _singulateRight;
    private final CANSparkMax _indexLower;
    private final CANSparkMax _indexUpper;
    private final DigitalInput _lowerBallSensor;
    private final DigitalInput _upperBallSensor;
    private final DoubleSolenoid _intakeSolenoid;

    private boolean _wasBallFed;
    private boolean _previousLowerSensorState;
    private boolean _previousUpperSensorState;
    private boolean _currentLowerSensorState;
    private boolean _currentUpperSensorState;

    private IntakeStates _currentState;


    private Double firstUpperSensorNotSenseTime = null;

    public IntakeSubsystem(DoubleSolenoid intakeSolenoid) {
        _intake = new CANSparkMax(Constants.CanIds.INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);
        _singulateLeft = new CANSparkMax(Constants.CanIds.SINGULATE_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
        _singulateRight = new CANSparkMax(Constants.CanIds.SINGULATE_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
        _indexLower = new CANSparkMax(Constants.CanIds.INDEX_LOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _indexUpper = new CANSparkMax(Constants.CanIds.INDEX_UPPER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _lowerBallSensor = new DigitalInput(Constants.DigitalIO.LOWER_BALL_SENSOR);
        _upperBallSensor = new DigitalInput(Constants.DigitalIO.UPPER_BALL_SENSOR);
        _intakeSolenoid = intakeSolenoid;
    
        // Set brake mode for all motors for better control over ball movement
        for (CANSparkMax spark : new CANSparkMax[] {_intake, _singulateLeft, _singulateRight, _indexUpper, _indexLower }) {
            spark.restoreFactoryDefaults();
            spark.setIdleMode(IdleMode.kBrake);
        }
         
        // we assume that both singulate motors are going to have exactly the same speed
        _singulateRight.follow(_singulateLeft, true);

        // The goal here is to make it so that calls to set(double) with positive values suck the ball in
        for (CANSparkMax spark : new CANSparkMax[] {_intake, _singulateLeft, _indexUpper, _indexLower }) {
            spark.setInverted(true);
        }

        // Set a slight ramp on these motors since they are geared low and quick reversals are likely
        for (CANSparkMax spark : new CANSparkMax[] {_intake, _singulateLeft, _singulateRight }) {
            spark.setOpenLoopRampRate(Constants.IntakeConstants.INTAKE_RAMP_RATE);
        }
        // Set a slight ramp on the index motors to let the ball coast a bit
        for (CANSparkMax spark : new CANSparkMax[] {_indexLower, _indexUpper }) {
            //spark.setOpenLoopRampRate(Constants.IntakeConstants.INDEX_RAMP_RATE);
        }
        

        // Start with no balls
        _previousLowerSensorState = false;
        _previousUpperSensorState = false;
        _currentLowerSensorState = false;
        _currentUpperSensorState = false;
        _currentState = IntakeStates.Idle;
        _wasBallFed = false;
        _intakeSolenoid.set(Constants.IntakeConstants.INTAKE_UP);
    }

    @Override
    public void periodic() {
        _previousLowerSensorState = _currentLowerSensorState;
        _previousUpperSensorState = _currentUpperSensorState;

        _currentLowerSensorState = lowerBallPresent();
        _currentUpperSensorState = upperBallPresent();

        if (DriverStation.isDisabled()) {
            stop(false);
        }
        update();
    }

    private void update() {
        switch (_currentState) {
            case Idle:
                this.idle();
            break;

            case Stop:
                this.stop(true);
            break;

            case Intake:
                this.intake();
            break;

            case Eject:
                this.eject(true);
            break;

            case EjectLower:
                this.eject(false);
            break;

            case FeedShooter:
                this.feed();
            break;
        }
    }

    /**
     * Lower the intake mechanism but don't start the motors
     */
    public void lowerIntake() {
        _intakeSolenoid.set(DoubleSolenoid.Value.kForward);
        _currentState = IntakeStates.Idle;
    }

    /**
     * Raise the intake mechanism and stop all motion
     */
    public void raiseIntake() { 
        this.stop(false);
        _intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Stop all motion on the intake subsystem
     * @param keepStopped If true, intake state will be set to Stop and no motion will occur until operator gives another input.
     * If false, all motors will stop momentarily, but state will be set to Idle in which the upper indexer may continue to move
     * in order to keep a ball loaded in the shooter.
     */
    public void stop(boolean keepStopped) {
        _intake.set(IntakeControl.Stop);
        _singulateLeft.set(IntakeControl.Stop);
        _indexLower.set(IntakeControl.Stop);
        _indexUpper.set(IntakeControl.Stop);

        _currentState = keepStopped ? IntakeStates.Stop : IntakeStates.Idle;
    }

    public void setStopState(boolean keepStopped) {
        _currentState = keepStopped ? IntakeStates.Stop : IntakeStates.Idle;
    }

    /**
     * Set the intake to eject balls
     * @param ejectAll If true, all intake, singulate, and index motors will eject. If false, only the lower intake and singulate motors will eject
     * Eject is only allowed with the intake down otherwise weird things may happen
     */
    public void eject(boolean ejectAll) {
        if (_intakeSolenoid.get() == Constants.IntakeConstants.INTAKE_DOWN)
        {
            if (ejectAll) {
                this.ejectAll();
            }
            else {
                this.ejectLower();
            }
        }
    }

    /**
     * Reverses all motors to remove all balls from system
     */
    private void ejectAll() { 
        _intake.set(IntakeControl.Intake_Eject);
        _singulateLeft.set(IntakeControl.Singulate_Eject);
        _indexLower.set(IntakeControl.Lower_Index_Eject);
        _indexUpper.set(IntakeControl.Upper_Index_Eject);
        _currentState = IntakeStates.Eject;
    }

    /**
     * Reverse all motors except upper indexer. This could be used to remove a ball of the wrong color
     */
    private void ejectLower() {
        _intake.set(IntakeControl.Intake_Eject);
        _singulateLeft.set(IntakeControl.Singulate_Eject);
        _indexLower.set(IntakeControl.Lower_Index_Eject);
        _indexUpper.set(IntakeControl.Stop);
        _currentState = IntakeStates.EjectLower;
    }

    /**
     * Main intake function. Controls motors in conjunction with ball detection sensors.
     */
    public void intake() {

        // If the intake is up and the operator wants to intake, lower the intake first
        if (_intakeSolenoid.get() == Constants.IntakeConstants.INTAKE_UP) {
            this.lowerIntake();
        }

        // If there are balls in both the upper and lower indexers, stop everything
        if (_currentLowerSensorState && _currentUpperSensorState) {
            this.stop(false);
        }
        // If there is a ball in the upper index, but nowhere else, stop the upper indexer and keep running everything else
        else if (!_currentLowerSensorState && _currentUpperSensorState) {
            _intake.set(IntakeControl.Intake_Intake);
            _singulateLeft.set(IntakeControl.Singulate_Intake);
            _indexLower.set(IntakeControl.Lower_Index_Intake);
            _indexUpper.set(IntakeControl.Stop);
        }
        // In all other scenarios, run all the intake motors
        else {
            this.intakeAll();
        }

        _currentState = IntakeStates.Intake;
    }

    public void feed() {
        _wasBallFed = false;

        if (!_currentUpperSensorState) {
            _indexLower.set(IntakeControl.Lower_Index_Intake);
        }
        else {
            _indexLower.set(IntakeControl.Stop);
        }

        if (_previousUpperSensorState == _currentUpperSensorState) {
            _indexUpper.set(IntakeControl.Upper_Index_Feed);
        } else {
            _indexUpper.set(IntakeControl.Stop);
            _wasBallFed = true;
        }  
        _currentState = IntakeStates.Idle;
    }

    public boolean wasBallFed() {
        return _wasBallFed;
    }

    public void clearBallFed() {
        _wasBallFed = false;
    }

    /**
     * Runs all motors in the intake direction.
     */
    private void intakeAll() {
        _intake.set(IntakeControl.Intake_Intake);
        _singulateLeft.set(IntakeControl.Singulate_Intake);
        _indexLower.set(IntakeControl.Lower_Index_Intake);
        _indexUpper.set(IntakeControl.Upper_Index_Intake);
    }

    /**
     * If the upper indexer is empty, run both indexers to move a ball up to the shooter
     */
    private void idle() {
        if (!_currentUpperSensorState) {
            _indexLower.set(IntakeControl.Lower_Index_Intake);
            _indexUpper.set(IntakeControl.Upper_Index_Intake);
        }
        else {
            this.stop(false);
        }
    }

    private boolean lowerBallPresent() {
        return !_lowerBallSensor.get();
    }

    private boolean upperBallPresent() {
        boolean sensed = !_upperBallSensor.get();
        if (!sensed) {
            double now = Timer.getFPGATimestamp();
            if (firstUpperSensorNotSenseTime == null) {
                firstUpperSensorNotSenseTime = now;
            }
            double timeNotSensed = now - firstUpperSensorNotSenseTime;
            return !(timeNotSensed >= 0.1);
        }
        firstUpperSensorNotSenseTime = null;
        return true;
    }
}
