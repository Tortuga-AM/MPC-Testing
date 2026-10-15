package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeometrySample {
    final Translation2d position;
    final Rotation2d tangent;
    final double curvatureRadPerMeter;
    final double distanceMeters;

    double maxVelocityMetersPerSecond;
    double velocityMetersPerSecond;
    double timeSeconds;

    /*
     * Initialized later by sampleBezierGeometry's caller.
     */
    double maxAccelerationMetersPerSecondSq =
        AutoAlignConstants.MAX_LINEAR_ACCEL_METERS_PER_SECOND_SQ;

    GeometrySample(
        Translation2d position,
        Rotation2d tangent,
        double curvatureRadPerMeter,
        double distanceMeters
    ) {
        this.position = position;
        this.tangent = tangent;
        this.curvatureRadPerMeter = curvatureRadPerMeter;
        this.distanceMeters = distanceMeters;
    }
}