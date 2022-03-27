package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform2d;

public final class Surrounding {
    private final Transform2d _transform;
    private final double _timestamp;

    public Surrounding(Transform2d transform, double timestamp) {
        _transform = transform;
        _timestamp = timestamp;
    }

    public Transform2d getTransform() {
        return _transform;
    }

    public double getTimestamp() {
        return _timestamp;
    }

    @Override
    public String toString() {
        return "Surrounding(" +
                "transform=" + _transform +
                ", timestamp=" + _timestamp +
                ')';
    }
}
