package frc.robot.Drive.AutoAlign;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Drive.TunerConstants;

/**
 * All easily-tuned AutoAlign/planning/trajectory constants.
 *
 * <p>Keep conservative values until the planner, collision checker,
 * trajectory generator, and follower have been tested thoroughly.
 */
public final class AutoAlignConstants {
    private AutoAlignConstants() {}

    // =========================================================================
    // Planner
    // =========================================================================

    /**
     * This is informational while loading PathPlanner navgrid.json.
     *
     * The actual grid resolution comes from navgrid.json's nodeSizeMeters field.
     * To use 0.10 m, regenerate that file with nodeSizeMeters = 0.10.
     */
    public static final double DESIRED_NAVGRID_NODE_SIZE_METERS = 0.10;

    /** Initial weighted-search heuristic factor. */
    public static final double ADSTAR_INITIAL_EPSILON = 2.5;

    /** Epsilon reduction after each refinement pass. */
    public static final double ADSTAR_EPSILON_STEP = 0.5;

    /** Maximum cells allowed during extraction before declaring invalid. */
    public static final int ADSTAR_MAX_PATH_EXTRACTION_STEPS = 2000;

    /** Background planner idle delay. */
    public static final long PLANNER_IDLE_SLEEP_MILLIS = 5;

    /** Timeout while waiting for the background planner. */
    public static final double PLANNING_TIMEOUT_SECONDS = 2.0;

    // =========================================================================
    // Bézier Geometry / Collision Checking
    // =========================================================================

    public static final double[] BEZIER_HANDLE_SCALES = {
        1.00,
        0.75,
        0.50,
        0.25,
        0.00
    };

    public static final double BEZIER_CORNER_ANCHOR_PERCENT = 0.80;
    public static final double BEZIER_MAX_CORNER_CUT_PERCENT = 0.40;

    /** Spacing used to test Bézier geometry against occupancy cells. */
    public static final double BEZIER_COLLISION_SAMPLE_SPACING_METERS = 0.02;

    /** Spacing used for path sampling during trajectory generation. */
    public static final double TRAJECTORY_SAMPLE_SPACING_METERS = 0.04;

    // =========================================================================
    // Trajectory Constraints
    // =========================================================================

    public static final double MAX_LINEAR_SPEED_METERS_PER_SECOND = 4.5;
    public static final double MAX_LINEAR_ACCEL_METERS_PER_SECOND_SQ = 11.5;
    public static final double MAX_LATERAL_ACCEL_METERS_PER_SECOND_SQ = 11.5;

    public static final double MAX_ANGULAR_SPEED_RAD_PER_SECOND =
        Math.toRadians(360.0);

    public static final double MAX_ANGULAR_ACCEL_RAD_PER_SECOND_SQ =
        Math.toRadians(540.0);

    /**
     * Maximum desired module drive acceleration.
     *
     * This is a trajectory/setpoint constraint, not necessarily the physical
     * motor maximum. Start conservatively.
     */
    public static final double MAX_MODULE_ACCEL_METERS_PER_SECOND_SQ = 7.0;

    /**
     * Maximum module azimuth steering velocity.
     *
     * Tune with real measured module behavior. A conservative initial value.
     */
    public static final double MAX_MODULE_STEER_RATE_RAD_PER_SECOND =
        Math.toRadians(720.0);

    public static final double CONTROL_LOOP_PERIOD_SECONDS = 0.020;

    public static final double MAX_MODULE_SPEED_METERS_PER_SECOND =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    // =========================================================================
    // Robot Geometry
    // =========================================================================

    public static final Translation2d[] MODULE_LOCATIONS = {
        new Translation2d(
            Inches.of(10.9).in(edu.wpi.first.units.Units.Meters),
            Inches.of(10.9).in(edu.wpi.first.units.Units.Meters)
        ),
        new Translation2d(
            Inches.of(10.9).in(edu.wpi.first.units.Units.Meters),
            Inches.of(-10.9).in(edu.wpi.first.units.Units.Meters)
        ),
        new Translation2d(
            Inches.of(-10.9).in(edu.wpi.first.units.Units.Meters),
            Inches.of(10.9).in(edu.wpi.first.units.Units.Meters)
        ),
        new Translation2d(
            Inches.of(-10.9).in(edu.wpi.first.units.Units.Meters),
            Inches.of(-10.9).in(edu.wpi.first.units.Units.Meters)
        )
    };

    // =========================================================================
    // Follower / Finish Detection
    // =========================================================================

    public static final double TRANSLATION_KP = 3.5;
    public static final double TRANSLATION_KI = 0.0;
    public static final double TRANSLATION_KD = 0.15;

    public static final double ROTATION_KP = 4.5;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.10;

    public static final double POSITION_TOLERANCE_METERS = 0.07;
    public static final double ROTATION_TOLERANCE_RADIANS =
        Math.toRadians(3.0);

    public static final double SETTLED_TIME_SECONDS = 0.15;

    // =========================================================================
    // Initial / End Velocity
    // =========================================================================

    /**
     * If measured initial speed is above this value but points opposite the
     * newly planned path direction, begin with zero path speed instead.
     */
    public static final double INITIAL_SPEED_DIRECTION_DOT_THRESHOLD = 0.0;

    /**
     * Default terminal translation speed for align-to-pose requests.
     * Use zero for a scoring/alignment pose.
     */
    public static final double DEFAULT_END_VELOCITY_METERS_PER_SECOND = 0.0;

    // =========================================================================
    // Generated Field Map
    // =========================================================================

    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.02;

    /*
    * 0.15 m gives approximately 111 x 54 = 5,994 grid cells.
    *
    * This is a good middle ground between PathPlanner's usual 0.20 m grid and
    * a more expensive 0.10 m grid.
    */
    public static final double NAVGRID_NODE_SIZE_METERS = 0.15;

    // =========================================================================
    // Segmented Paths
    // =========================================================================

    /*
    * Used only when RequiredWaypoint.stopAtWaypoint() is false and the caller
    * does not specify a custom pass-through velocity.
    */
    public static final double DEFAULT_PASS_THROUGH_VELOCITY_METERS_PER_SECOND =
        1.0;

    /*
    * AutoAlign considers a waypoint reached when the robot is within this
    * translation/heading tolerance. A non-stop waypoint transitions to the next
    * plan immediately after this is satisfied.
    */
    public static final double WAYPOINT_POSITION_TOLERANCE_METERS = 0.12;
    public static final double WAYPOINT_ROTATION_TOLERANCE_RADIANS =
        Math.toRadians(8.0);

    // =========================================================================
    // Disturbance / Stuck Detection
    // =========================================================================

    public static final double REPLAN_CROSS_TRACK_ERROR_METERS = 0.45;
    public static final double REPLAN_GRACE_PERIOD_SECONDS = 0.30;
    public static final double MIN_REPLAN_INTERVAL_SECONDS = 0.35;

    /*
    * Stuck means the controller has asked for meaningful movement but the robot
    * has not actually moved enough for a sustained period.
    */
    public static final double STUCK_COMMAND_SPEED_THRESHOLD_METERS_PER_SECOND =
        0.75;

    public static final double STUCK_MEASURED_SPEED_THRESHOLD_METERS_PER_SECOND =
        0.15;

    public static final double STUCK_MIN_PROGRESS_METERS = 0.04;
    public static final double STUCK_DETECTION_TIME_SECONDS = 0.50;

    /*
    * If a trajectory's scheduled time expires and the robot does not reach its
    * target after this additional amount of time, replan.
    */
    public static final double MAX_TRAJECTORY_OVERRUN_SECONDS = 1.50;

    // =========================================================================
    // Actionable Zones
    // =========================================================================

    public static final double ACTIONABLE_ZONE_DEBOUNCE_SECONDS = 0.10;
}