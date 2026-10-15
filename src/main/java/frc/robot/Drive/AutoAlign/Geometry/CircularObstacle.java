package frc.robot.Drive.AutoAlign.Geometry;

import edu.wpi.first.math.geometry.Translation2d;

public class CircularObstacle implements Obstacle {
    private final Translation2d center;
    private final double radiusMeters;

    public CircularObstacle(Translation2d center, double radiusMeters) {
        if (radiusMeters <= 0.0) {
            throw new IllegalArgumentException("Circle radius must be positive");
        }

        this.center = center;
        this.radiusMeters = radiusMeters;
    }

    public CircularObstacle(Translation2d center, double radiusMeters, double inflationRadius) {
        this(center, radiusMeters + inflationRadius);
    }


    @Override
    public boolean contains(Translation2d point) {
        return point.getDistance(center) <= radiusMeters;
    }

    @Override
    public Bounds getBounds() {
        return new Bounds(
            center.getX() - radiusMeters,
            center.getX() + radiusMeters,
            center.getY() - radiusMeters,
            center.getY() + radiusMeters
        );
    }
}