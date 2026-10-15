package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;
import frc.robot.Drive.AutoAlign.Geometry.Obstacle;
import frc.robot.Drive.AutoAlign.Geometry.Bounds;

/**
 * Rasterized occupancy grid generated entirely from field dimensions and
 * geometric obstacle shapes.
 *
 * <p>Coordinates use normal WPILib field coordinates:
 *
 * <ul>
 *   <li>X: field length direction
 *   <li>Y: field width direction
 * </ul>
 */
public class FieldObstacleMap {
    private final double fieldLengthMeters;
    private final double fieldWidthMeters;
    private final double nodeSizeMeters;

    private final int nodesX;
    private final int nodesY;

    private final boolean[][] occupied;

    public FieldObstacleMap(
        double fieldLengthMeters,
        double fieldWidthMeters,
        double nodeSizeMeters,
        List<Obstacle> obstacles
    ) {
        if (fieldLengthMeters <= 0.0
            || fieldWidthMeters <= 0.0
            || nodeSizeMeters <= 0.0) {
            throw new IllegalArgumentException(
                "Field dimensions and node size must be positive"
            );
        }

        this.fieldLengthMeters = fieldLengthMeters;
        this.fieldWidthMeters = fieldWidthMeters;
        this.nodeSizeMeters = nodeSizeMeters;

        nodesX = (int) Math.ceil(fieldLengthMeters / nodeSizeMeters);
        nodesY = (int) Math.ceil(fieldWidthMeters / nodeSizeMeters);

        occupied = new boolean[nodesX][nodesY];

        rasterize(obstacles == null ? List.of() : obstacles);
    }

    /**
     * Creates an empty field map. Useful for debugging whether AD* itself works.
     */
    public static FieldObstacleMap empty(
        double fieldLengthMeters,
        double fieldWidthMeters,
        double nodeSizeMeters
    ) {
        return new FieldObstacleMap(
            fieldLengthMeters,
            fieldWidthMeters,
            nodeSizeMeters,
            List.of()
        );
    }

    private void rasterize(List<Obstacle> obstacles) {
        for (Obstacle obstacle : obstacles) {
            Bounds bounds = obstacle.getBounds();

            int minX = clampX(
                (int) Math.floor(bounds.minX() / nodeSizeMeters)
            );

            int maxX = clampX(
                (int) Math.floor(bounds.maxX() / nodeSizeMeters)
            );

            int minY = clampY(
                (int) Math.floor(bounds.minY() / nodeSizeMeters)
            );

            int maxY = clampY(
                (int) Math.floor(bounds.maxY() / nodeSizeMeters)
            );

            for (int x = minX; x <= maxX; x++) {
                for (int y = minY; y <= maxY; y++) {
                    Translation2d cellCenter = toFieldPosition(x, y);

                    if (obstacle.contains(cellCenter)) {
                        occupied[x][y] = true;
                    }
                }
            }
        }
    }

    public boolean isOccupied(int x, int y) {
        return !isInBounds(x, y) || occupied[x][y];
    }

    public boolean isInBounds(int x, int y) {
        return x >= 0 && x < nodesX && y >= 0 && y < nodesY;
    }

    public Translation2d toFieldPosition(int x, int y) {
        return new Translation2d(
            (x + 0.5) * nodeSizeMeters,
            (y + 0.5) * nodeSizeMeters
        );
    }

    public int toGridX(double fieldX) {
        return (int) Math.floor(fieldX / nodeSizeMeters);
    }

    public int toGridY(double fieldY) {
        return (int) Math.floor(fieldY / nodeSizeMeters);
    }

    public int getNodesX() {
        return nodesX;
    }

    public int getNodesY() {
        return nodesY;
    }

    public double getNodeSizeMeters() {
        return nodeSizeMeters;
    }

    public double getFieldLengthMeters() {
        return fieldLengthMeters;
    }

    public double getFieldWidthMeters() {
        return fieldWidthMeters;
    }

    private int clampX(int x) {
        return Math.max(0, Math.min(nodesX - 1, x));
    }

    private int clampY(int y) {
        return Math.max(0, Math.min(nodesY - 1, y));
    }
}