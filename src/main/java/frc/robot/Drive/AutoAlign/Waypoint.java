package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A required waypoint in a multi-waypoint AutoAlign request.
 *
 * If stopAtWaypoint is true, the trajectory reaches this pose at zero speed.
 * If false, AutoAlign requests a nonzero pass-through speed and immediately
 * plans/follows the next leg after reaching the waypoint tolerance.
 */
public record Waypoint(
    Pose2d pose,
    boolean stopAtWaypoint,
    double passThroughVelocityMetersPerSecond,
    Runnable onReached
) {
    public Waypoint {
        if (passThroughVelocityMetersPerSecond < 0.0) {
            throw new IllegalArgumentException(
                "passThroughVelocityMetersPerSecond cannot be negative"
            );
        }

        onReached = onReached == null ? () -> {} : onReached;
    }

    public static Waypoint stop(Pose2d pose, Runnable onReached) {
        return new Waypoint(pose, true, 0.0, onReached);
    }

    public static Waypoint passThrough(
        Pose2d pose,
        double velocityMetersPerSecond,
        Runnable onReached
    ) {
        return new Waypoint(
            pose,
            false,
            velocityMetersPerSecond,
            onReached
        );
    }
}