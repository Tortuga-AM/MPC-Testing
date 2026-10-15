package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A region that limits trajectory velocity and acceleration.
 *
 * Multiple overlapping constraint zones combine conservatively: the lowest
 * velocity and acceleration limits win.
 */
public class ConstraintZone implements Zone {
    private final String name;
    private final Zone shape;

    private final double maxVelocityMetersPerSecond;
    private final double maxAccelerationMetersPerSecondSq;
    private final double maxLateralAccelerationMetersPerSecondSq;

    public ConstraintZone(
        String name,
        Zone shape,
        double maxVelocityMetersPerSecond,
        double maxAccelerationMetersPerSecondSq,
        double maxLateralAccelerationMetersPerSecondSq
    ) {
        if (maxVelocityMetersPerSecond <= 0.0
            || maxAccelerationMetersPerSecondSq <= 0.0
            || maxLateralAccelerationMetersPerSecondSq <= 0.0) {
            throw new IllegalArgumentException(
                "Constraint-zone limits must all be positive"
            );
        }

        this.name = name;
        this.shape = shape;
        this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
        this.maxAccelerationMetersPerSecondSq =
            maxAccelerationMetersPerSecondSq;
        this.maxLateralAccelerationMetersPerSecondSq =
            maxLateralAccelerationMetersPerSecondSq;
    }

    @Override
    public boolean contains(Translation2d position) {
        return shape.contains(position);
    }

    @Override
    public String getName() {
        return name;
    }

    public double getMaxVelocityMetersPerSecond() {
        return maxVelocityMetersPerSecond;
    }

    public double getMaxAccelerationMetersPerSecondSq() {
        return maxAccelerationMetersPerSecondSq;
    }

    public double getMaxLateralAccelerationMetersPerSecondSq() {
        return maxLateralAccelerationMetersPerSecondSq;
    }
}