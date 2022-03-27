package frc.robot.vision;

import org.jetbrains.annotations.NotNull;

import java.util.List;

import static java.util.Objects.requireNonNull;

public final class VisionInstant {
    private final List<Surrounding> surroundings;
    private final double timestamp;

    public VisionInstant(List<Surrounding> surroundings, double timestamp) {
        this.surroundings = requireNonNull(surroundings);
        this.timestamp = timestamp;
    }

    public @NotNull List<Surrounding> getSurroundings() {
        return surroundings;
    }

    public double getTimestamp() {
        return timestamp;
    }
}
