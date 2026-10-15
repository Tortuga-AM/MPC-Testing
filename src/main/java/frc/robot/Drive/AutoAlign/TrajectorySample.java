package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * One time-stamped desired state in a custom holonomic trajectory.
 *
 * fieldVelocityMetersPerSecond is field-relative.
 * angularVelocityRadPerSecond is positive CCW.
 */
public record TrajectorySample (
    double timeSeconds,
    Pose2d pose,
    Translation2d fieldVelocityMetersPerSecond,
    double angularVelocityRadPerSecond
) {}