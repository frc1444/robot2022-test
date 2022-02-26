package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private static final double MAX_SPEED_INTAKE = 0.25; // TODO adjust this
    private static final double MAX_SPEED_SINGULATE = 0.25;
    private static final double MAX_SPEED_INDEX = 0.25; // TODO consider having different max speeds for upper and lower motors

    private final CANSparkMax intake;
    private final CANSparkMax singulateLeft;
    private final CANSparkMax indexLower;
    private final CANSparkMax indexUpper;

    // TODO pneumatics stuff for raising and lowering intake
    // TODO sensor stuff for detecting balls


    public IntakeSubsystem() {
        intake = new CANSparkMax(Constants.CanIds.INTAKE, CANSparkMaxLowLevel.MotorType.kBrushless);
        singulateLeft = new CANSparkMax(Constants.CanIds.SINGULATE_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
        CANSparkMax singulateRight = new CANSparkMax(Constants.CanIds.SINGULATE_RIGHT, CANSparkMaxLowLevel.MotorType.kBrushless);
        indexLower = new CANSparkMax(Constants.CanIds.INDEX_LOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
        indexUpper = new CANSparkMax(Constants.CanIds.INDEX_UPPER, CANSparkMaxLowLevel.MotorType.kBrushless);

        // The goal here is to make it so that calls to set(double) with positive values suck the ball in
        // TODO figure out which motors need a setInverted(true) call

        singulateRight.follow(singulateLeft, true); // we assume that both singulate motors are going to have exactly the same speed
    }

    public void update(double intakeSpeed, double indexSpeed) {
        // Right now we have the singulate motors to run when we intake. We can change this later if we need to.
        //   We may consider adding ball detection logic here. Because of the communication needed between
        //   the intake and the indexing, we combine intake and indexing into one subsystem.
        //   If we find that we can separate intake and indexing logic, we could add the IndexSubsystem back

        // This update method will likely be changed, especially when we need to do stuff with the automatic indexing of balls.
        //   We will probably do something similar to robot2020 for that.

        intake.set(intakeSpeed * MAX_SPEED_INTAKE);
        singulateLeft.set(intakeSpeed * MAX_SPEED_SINGULATE);
        // singulateRight is following singulateLeft, so no need to set it here

        // We don't have indexUpper follow indexLower because we may set them to different speeds in the future
        indexLower.set(indexSpeed * MAX_SPEED_INDEX);
        indexUpper.set(indexSpeed * MAX_SPEED_INDEX);
    }

}
