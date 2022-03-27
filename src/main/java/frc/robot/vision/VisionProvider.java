package frc.robot.vision;

import org.jetbrains.annotations.Nullable;

public interface VisionProvider {
    @Nullable
    VisionInstant getVisionInstant();

    VisionProvider NOTHING = () -> null;
}
