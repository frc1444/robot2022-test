package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerAny {

    public static Trigger create(Trigger... triggers) {
        return new Trigger(() -> {
            for (Trigger trigger : triggers) {
                if (trigger.get()) {
                    return true;
                }
            }
            return false;
        });
    }
}
