package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * User-defined desired robot-heading function.
 *
 * <p>This runs while generating a trajectory, not every robot loop.
 */
@FunctionalInterface
public interface HeadingFunction {
    Rotation2d getHeading(HeadingContext context);
}