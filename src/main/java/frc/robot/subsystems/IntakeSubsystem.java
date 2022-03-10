package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax _intake;
    private final CANSparkMax _singulateLeft;
    private final CANSparkMax _singulateRight;
    private final CANSparkMax _indexLower;
    private final CANSparkMax _indexUpper;
    private final DigitalInput _lowerBallSensor;
    private final DigitalInput _upperBallSensor;

    private int _ballCount;
    private boolean _previousLowerSensorState;
    private boolean _previousUpperSensorState;
    private boolean _currentLowerSensorState;
    private boolean _currentUpperSensorState;

    private IntakeStates _currentState;

    // TODO pneumatics stuff for raising and lowering intake


    public IntakeSubsystem() {
        _intake = new CANSparkMax(Constants.CanIds.INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);
        _singulateLeft = new CANSparkMax(Constants.CanIds.SINGULATE_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
        _singulateRight = new CANSparkMax(Constants.CanIds.SINGULATE_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
        _indexLower = new CANSparkMax(Constants.CanIds.INDEX_LOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _indexUpper = new CANSparkMax(Constants.CanIds.INDEX_UPPER, CANSparkMaxLowLevel.MotorType.kBrushless);
        _lowerBallSensor = new DigitalInput(Constants.DigitalIO.LOWER_BALL_SENSOR);
        _upperBallSensor = new DigitalInput(Constants.DigitalIO.UPPER_BALL_SENSOR);
    
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
            spark.setOpenLoopRampRate(0.3);
        }

        // Start with no balls
        _ballCount = 0;
        _previousLowerSensorState = false;
        _previousUpperSensorState = false;
        _currentLowerSensorState = false;
        _currentUpperSensorState = false;
        _currentState = IntakeStates.Stop;
    }

    @Override
    public void periodic() {
        _previousLowerSensorState = _currentLowerSensorState;
        _previousUpperSensorState = _currentUpperSensorState;

        _currentLowerSensorState = lowerBallPresent();
        _currentUpperSensorState = upperBallPresent();
    }

    public void setState(IntakeStates state) {
        _currentState = state;
    }

    public void update() {
        switch (_currentState) {
            case Idle:
                this.moveBallUp();
            break;
            case Stop:
                this.stop();
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
     * Stop all motion on the intake subsystem
     */
    public void stop() {
        _intake.set(Constants.IntakeControl.STOP);
        _singulateLeft.set(Constants.IntakeControl.STOP);
        _indexLower.set(Constants.IntakeControl.STOP);
        _indexUpper.set(Constants.IntakeControl.STOP);
    }

    public void eject(boolean ejectAll) {
        if (ejectAll) {
            this.ejectAll();
        }
        else {
            this.ejectLower();
        }

        if (_previousLowerSensorState && !_currentLowerSensorState) {
            if (_ballCount > 0) {
                _ballCount -= 1;
            }
        }
    }

    /**
     * Reverses all motors to remove all balls from system
     */
    private void ejectAll() { 
        _intake.set(Constants.IntakeControl.INTAKE_EJECT);
        _singulateLeft.set(Constants.IntakeControl.SINGULATE_EJECT);
        _indexLower.set(Constants.IntakeControl.LOWER_INDEX_EJECT);
        _indexUpper.set(Constants.IntakeControl.UPPER_INDEX_EJECT);
    }

    /**
     * Reverse all motors except upper indexer. This could be used to remove a ball of the wrong color
     */
    private void ejectLower() {
        _intake.set(Constants.IntakeControl.INTAKE_EJECT);
        _singulateLeft.set(Constants.IntakeControl.SINGULATE_EJECT);
        _indexLower.set(Constants.IntakeControl.LOWER_INDEX_EJECT);
        _indexUpper.set(Constants.IntakeControl.STOP);
    }

    /**
     * Main intake function. Controls motors in conjunction with ball detection sensors.
     */
    public void intake() {

        // If there are balls in both the upper and lower indexers, stop everything
        if (_currentLowerSensorState && _currentUpperSensorState) {
            this.stop();
        }
        // If there is a ball in the upper index, but nowhere else, stop the upper indexer and keep running everything else
        else if (!_currentLowerSensorState && _currentUpperSensorState) {
            _intake.set(Constants.IntakeControl.INTAKE_INTAKE);
            _singulateLeft.set(Constants.IntakeControl.SINGULATE_INTAKE);
            _indexLower.set(Constants.IntakeControl.LOWER_INDEX_INTAKE);
            _indexUpper.set(Constants.IntakeControl.STOP);
        }
        // In all other scenarios, run all the intake motors
        else {
            this.intakeAll();
        }

        if (!_previousLowerSensorState && _currentLowerSensorState) {
            _ballCount += 1;
        }
    }

    public void feed() {
        if (_previousUpperSensorState == _currentUpperSensorState) {
            _indexUpper.set(Constants.IntakeControl.UPPER_INDEX_INTAKE);
        } else {
            _indexUpper.set(Constants.IntakeControl.STOP);
            
            if (_ballCount > 0) {
                _ballCount -= 1;
            }
        }  
    }

    /**
     * Runs all motors in the intake direction.
     */
    private void intakeAll() {
        _intake.set(Constants.IntakeControl.INTAKE_INTAKE);
        _singulateLeft.set(Constants.IntakeControl.SINGULATE_INTAKE);
        _indexLower.set(Constants.IntakeControl.LOWER_INDEX_INTAKE);
        _indexUpper.set(Constants.IntakeControl.UPPER_INDEX_INTAKE);
    }

    /**
     * If the upper indexer is empty and the lower index has a ball, move the ball to the upper indexer
     */
    public void moveBallUp() {
        if (_ballCount > 0 && !upperBallPresent()) {
            _indexLower.set(Constants.IntakeControl.LOWER_INDEX_INTAKE);
            _indexUpper.set(Constants.IntakeControl.UPPER_INDEX_INTAKE);
        }
        else {
            this.stop();
        }
    }

    public int getBallCount() {
        return _ballCount;
    }

    private boolean lowerBallPresent() {
        return !_lowerBallSensor.get();
    }

    private boolean upperBallPresent() {
        return !_upperBallSensor.get();
    }
}
