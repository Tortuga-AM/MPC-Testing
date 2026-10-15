package frc.robot.Drive.AutoAlign.Geometry;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

public class TriangularObstacle implements Obstacle {
    private final PolygonObstacle polygon;

    public TriangularObstacle(
        Translation2d first,
        Translation2d second,
        Translation2d third
    ) {
        polygon = new PolygonObstacle(List.of(first, second, third));
    }

    public TriangularObstacle(
        Translation2d first,
        Translation2d second,
        Translation2d third,
        double inflationRadius
    ) {
        polygon = new PolygonObstacle(List.of(first, second, third), inflationRadius);
    }

    @Override
    public boolean contains(Translation2d point) {
        return polygon.contains(point);
    }

    @Override
    public Bounds getBounds() {
        return polygon.getBounds();
    }
}