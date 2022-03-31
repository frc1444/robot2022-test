package frc.robot.autonomous;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public final class AutonomousCommandBuilder {
    
    public static Command buildAutoCommand(
        String name, 
        Trajectory trajectory,
        DriveSubsystem drive,
        ShooterSubsystem shooter,
        IntakeSubsystem intake) {

        CommandBase cmd;

        switch (name) {
            case "two ball south":
            case "two ball south west":
            case "one ball":
                cmd = new TwoBallExample(drive, intake, shooter, trajectory);
            break;
            
            case "do nothing":
                cmd = new DoNothing(drive, intake, shooter);
            break;

            default:
                cmd = new DoNothing(drive, intake, shooter);
            break;
        }

        cmd.setName(name);
        return cmd;
    }
}
