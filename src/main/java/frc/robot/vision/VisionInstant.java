package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.List;

import static java.util.Objects.requireNonNull;

public final class VisionInstant {
    private final List<Surrounding> surroundings;
    private final Rotation2d averageTheta;
    private final double timestamp;

    public VisionInstant(List<Surrounding> surroundings, Rotation2d averageTheta, double timestamp) {
        this.surroundings = requireNonNull(surroundings);
        this.averageTheta = averageTheta;
        this.timestamp = timestamp;
    }

    public @NotNull List<Surrounding> getSurroundings() {
        return surroundings;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public @Nullable Rotation2d getAverageTheta() {
        return averageTheta;
    }
}
