package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;

public class CommandUtil {
    private CommandUtil() { throw new UnsupportedOperationException(); }

    public static Command runWhenDisabled(Command command) {
        return new Command() {
            @Override
            public Set<Subsystem> getRequirements() {
                return command.getRequirements();
            }

            @Override
            public void initialize() {
                command.initialize();
            }

            @Override
            public void execute() {
                command.execute();
            }

            @Override
            public void end(boolean interrupted) {
                command.end(interrupted);
            }

            @Override
            public boolean isFinished() {
                return command.isFinished();
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }

            @Override
            public String getName() {
                return command.getName();
            }
        };
    }
}
