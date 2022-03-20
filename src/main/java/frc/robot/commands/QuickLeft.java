package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

public class QuickLeft extends TurnToAngle {
    public QuickLeft(DriveSubsystem drive) {
        super(
            calculateSetpoint(drive.getHeading() - 90),
            drive
        );
    }       
}
