package frc.robot.Drive.AutoAlign.Geometry;

import edu.wpi.first.math.geometry.Translation2d;

public class EllipticalObstacle implements Obstacle {
    private final Translation2d center;
    private final double radiusXMeters;
    private final double radiusYMeters;

    public EllipticalObstacle(
        Translation2d center,
        double radiusXMeters,
        double radiusYMeters
    ) {
        if (radiusXMeters <= 0.0 || radiusYMeters <= 0.0) {
            throw new IllegalArgumentException("Ellipse radii must be positive");
        }

        this.center = center;
        this.radiusXMeters = radiusXMeters;
        this.radiusYMeters = radiusYMeters;
    }

    public EllipticalObstacle(
        Translation2d center,
        double radiusXMeters,
        double radiusYMeters,
        double inflationRadius
    ) {
        this(center, radiusXMeters + inflationRadius, radiusYMeters + inflationRadius);
    }

    @Override
    public boolean contains(Translation2d point) {
        double normalizedX =
            (point.getX() - center.getX()) / radiusXMeters;

        double normalizedY =
            (point.getY() - center.getY()) / radiusYMeters;

        return (normalizedX * normalizedX)
            + (normalizedY * normalizedY) <= 1.0;
    }

    @Override
    public Bounds getBounds() {
        return new Bounds(
            center.getX() - radiusXMeters,
            center.getX() + radiusXMeters,
            center.getY() - radiusYMeters,
            center.getY() + radiusYMeters
        );
    }
}