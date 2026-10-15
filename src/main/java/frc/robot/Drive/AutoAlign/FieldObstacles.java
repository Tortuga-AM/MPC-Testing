package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Drive.AutoAlign.Geometry.CircularObstacle;
import frc.robot.Drive.AutoAlign.Geometry.EllipticalObstacle;
import frc.robot.Drive.AutoAlign.Geometry.Obstacle;
import frc.robot.Drive.AutoAlign.Geometry.PolygonObstacle;
import frc.robot.Drive.AutoAlign.Geometry.RectangularObstacle;
import frc.robot.Drive.AutoAlign.Geometry.TriangularObstacle;
import java.util.ArrayList;
import java.util.List;

/**
 * Defines all static field obstacles in field coordinates.
 *
 * <p>Coordinate system:
 * <ul>
 *   <li>X: blue alliance wall toward red alliance wall</li>
 *   <li>Y: left-to-right across the field</li>
 * </ul>
 *
 * <p>All obstacle shapes should already include robot-radius inflation.
 * For example, if a field element is physically 1.0 m wide and you want
 * 0.55 m of robot-center clearance, define its obstacle 0.55 m larger
 * on every relevant side.
 */
public final class FieldObstacles {
    private FieldObstacles() {}

    /**
     * Build static obstacles for the field.
     *
     * <p>This starts empty on purpose. Confirm that AD* works on an empty
     * generated grid first, then add field geometry one object at a time.
     */
    public static List<Obstacle> createStaticObstacles() {
        List<Obstacle> obstacles = new ArrayList<>();

        obstacles.add(
            new RectangularObstacle(
                new Translation2d(4.01, 1.32),
                new Translation2d(5.22, 6.72),
                0.5
            )
        );

        obstacles.add(
            new RectangularObstacle(
                new Translation2d(-0.01, 3.71),
                new Translation2d(1.18, 4.94),
                0.5
            )
        );

        obstacles.add(
            new RectangularObstacle(
                new Translation2d(11.29, 1.31),
                new Translation2d(12.54, 6.74),
                0.5
            )
        );

        obstacles.add(
            new RectangularObstacle(
                new Translation2d(15.37, 3.21),
                new Translation2d(16.49, 4.33),
                0.5
            )
        );

        return obstacles;
    }

    /**
     * Creates the one shared static occupancy map used by both AD* and the
     * Bézier collision checker.
     */
    public static FieldObstacleMap createFieldMap() {
        return new FieldObstacleMap(
            AutoAlignConstants.FIELD_LENGTH_METERS,
            AutoAlignConstants.FIELD_WIDTH_METERS,
            AutoAlignConstants.NAVGRID_NODE_SIZE_METERS,
            createStaticObstacles()
        );
    }
}