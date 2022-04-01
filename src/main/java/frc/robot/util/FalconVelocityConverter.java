package frc.robot.util;

import frc.robot.Constants;

public class FalconVelocityConverter {
    public static double rpmToVelocity(double rpm) {
        return rpm * Constants.FALCON_ENCODER_COUNTS_PER_REVOLUTION / Constants.CTRE_UNIT_CONVERSION;
    }
    public static double percentToVelocity(double percent) {
        return rpmToVelocity(percent * Constants.MAX_FALCON_RPM);
    }
}
