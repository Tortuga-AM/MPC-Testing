package frc.robot.Drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;

public class DriveConstants {

	public static final double SIM_UPDATE_TIME = 0.004;

	public static final Distance WHEEL_BASE = Meters.of(0.5);

	public static final double MIN_SCALE_FACTOR = 0.1;

	public static final double SLOW_MODE_MULTIPLIER = 0.75;

	public static final double CLOSE_TO_ZERO = Math.pow(10, -4);

	public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(11.7);

	public static final AngularVelocity ANGULAR_VELOCITY_LIMIT = AngularVelocity.ofBaseUnits(360, DegreesPerSecond);

	public static final LinearAcceleration MAX_LINEAR_DECELERATION = MetersPerSecondPerSecond.of(11);
	public static final LinearAcceleration MAX_LINEAR_STOPPING_ACCELERATION = MetersPerSecondPerSecond.of(10);

	public static final String SUBSYSTEM_NAME = "Drive";

	// For zeroing on robot init
	public static final Rotation2d BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0);
	public static final Rotation2d RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180);
}
