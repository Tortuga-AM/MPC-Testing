package frc.robot.Drive.AutoAlign.Geometry;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * A field-space obstacle shape.
 *
 * <p>The point provided to {@link #contains(Translation2d)} is normally a grid
 * cell center. Shapes should include any desired obstacle inflation before
 * being supplied to the map, or the map can apply a global clearance value.
 */
public interface Obstacle {
    double inflationRadius = 0.0;
    boolean contains(Translation2d point);
    /**
     * Conservative axis-aligned bounding box used to reduce rasterization work.
     */
    Bounds getBounds();
}