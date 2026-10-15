package frc.robot.Drive.AutoAlign.Geometry;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Axis-aligned rectangular field obstacle.
 */
public class RectangularObstacle implements Obstacle {
    private final Bounds bounds;

    public RectangularObstacle(
        Translation2d firstCorner,
        Translation2d oppositeCorner,
        double inflationRadius
    ) {
        this(
            Math.min(firstCorner.getX(), oppositeCorner.getX()) - inflationRadius,
            Math.max(firstCorner.getX(), oppositeCorner.getX()) + inflationRadius,
            Math.min(firstCorner.getY(), oppositeCorner.getY()) - inflationRadius,
            Math.max(firstCorner.getY(), oppositeCorner.getY()) + inflationRadius
        );
    }


    public RectangularObstacle(
        double minX,
        double maxX,
        double minY,
        double maxY
    ) {
        bounds = new Bounds(minX, maxX, minY, maxY);
    }

    public RectangularObstacle(
        Translation2d firstCorner,
        Translation2d oppositeCorner
    ) {
        this(
            Math.min(firstCorner.getX(), oppositeCorner.getX()),
            Math.max(firstCorner.getX(), oppositeCorner.getX()),
            Math.min(firstCorner.getY(), oppositeCorner.getY()),
            Math.max(firstCorner.getY(), oppositeCorner.getY())
        );
    }

    @Override
    public boolean contains(Translation2d point) {
        return bounds.contains(point.getX(), point.getY());
    }

    @Override
    public Bounds getBounds() {
        return bounds;
    }
}