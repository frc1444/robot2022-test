package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

public class QuickRight extends TurnToAngle {
    public QuickRight(DriveSubsystem drive) {
        super(
            calculateSetpoint(drive.getHeading() + 90),
            drive
        );
    }       
}
