package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Geometry at a point along a path.
 *
 * @param position Field-relative position
 * @param tangent Direction of translational travel
 * @param curvatureRadPerMeter Signed path curvature
 */
public record PathSample(
    Translation2d position,
    Rotation2d tangent,
    double curvatureRadPerMeter
) {}