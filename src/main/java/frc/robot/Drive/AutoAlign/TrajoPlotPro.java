package frc.robot.Drive.AutoAlign;

import static org.wpilib.math.autodiff.Variable.cos;
import static org.wpilib.math.autodiff.Variable.sin;
import static org.wpilib.math.optimization.Constraints.*;

import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.autodiff.VariableMatrix;
import org.wpilib.math.optimization.Problem;
import org.wpilib.math.optimization.solver.ExitStatus;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.PriorityQueue;

/**
 * Time-optimal-ish swerve trajectory optimizer built on a nonlinear Sleipnir problem.
 *
 * <p>State per knot: {@code [x, y, theta, vx, vy, omega]} (field frame).
 * Control per interval: {@code [Fx_0..Fx_(m-1), Fy_0..Fy_(m-1)]} (field frame).
 *
 * <h2>Design invariants</h2>
 * <ol>
 *   <li><b>One environment.</b> Keep-out geometry is built exactly once and is shared by the
 *       shunt, the geometric router, the NLP, and the warm start. Only final validation uses
 *       the raw physical geometry, deliberately, as an independent check.</li>
 *   <li><b>Compact keep-outs.</b> A rectangle is <i>one</i> smooth superellipse constraint,
 *       not a grid of circles. This is the single largest performance factor.</li>
 *   <li><b>No silent straight lines.</b> If geometric routing fails, it returns
 *       {@code Optional.empty()}. An unvalidated straight line is never substituted.</li>
 *   <li><b>Nothing unsafe escapes.</b> Every candidate trajectory — NLP or fallback — must
 *       pass {@link #validateTrajectory} before it is returned.</li>
 * </ol>
 *
 * <h2>Threading</h2>
 * A full solve takes hundreds of milliseconds to seconds. <b>Do not call this from a
 * command's {@code execute()} or from {@code robotPeriodic()}.</b> Run it on a single-thread
 * executor and consume the result when it becomes available.
 */
public final class TrajoPlotPro {

    // ─────────────────────────────────────────────────────────────────────────────
    // Tunables
    // ─────────────────────────────────────────────────────────────────────────────

    public static boolean ENABLE_LOGGING = true;

    private static final String LOG_FILE_PATH =
        Filesystem.getOperatingDirectory().getAbsolutePath()
            + File.separator + "TrajoPlotLogs.txt";

    /** Log file is truncated once it exceeds this size, so roboRIO flash cannot fill. */
    private static final long MAX_LOG_BYTES = 512L * 1024L;

    // Clearance budget. These stack, so keep them small and deliberate.
    private static final double NLP_EXTRA_CLEARANCE_M      = 0.010;
    private static final double SHUNT_CLEARANCE_M          = 0.060;
    private static final double GEOMETRIC_CLEARANCE_M      = 0.040;
    private static final double FIELD_EDGE_EXTRA_MARGIN_M  = 0.030;
    private static final double VALIDATION_TOLERANCE_M     = 0.005;
    private static final double VALIDATION_SPACING_M       = 0.025;

    // Discretization.
    private static final double MAX_KNOT_SPACING_M = 0.14;
    private static final double MAX_DT_S           = 0.08;
    private static final int    MIN_KNOT_INTERVALS = 16;
    private static final int    MAX_KNOT_INTERVALS = 160;

    /** Obstacle samples per knot interval. Interval motion is exactly linear under the
     *  semi-implicit integrator, so endpoints + midpoint is a tight check, not a guess. */
    private static final int NORMAL_OBSTACLE_SUBDIVISIONS = 2;
    private static final int DENSE_OBSTACLE_SUBDIVISIONS  = 4;

    /** Superellipse exponent for rectangles. Must be even. Higher = tighter fit, worse
     *  conditioning. 6 gives roughly 1% over-inflation on typical field geometry. */
    private static final int SUPERELLIPSE_EXPONENT = 6;

    private static final double MODEL_SPEED_SAFETY_FACTOR = 0.995;
    private static final double SANITIZE_SAFETY_FACTOR    = 0.98;
    private static final double WARM_START_LOOKAHEAD_M    = 0.20;

    /** Control-effort weight, applied to the <i>normalized</i> force so it is genuinely a
     *  tie-breaker against the time term rather than a competing objective. */
    private static final double EFFORT_WEIGHT = 1e-3;

    /** Enforce per-module wheel-speed limits. Physically important; costs ~4N constraints. */
    private static final boolean ENFORCE_MODULE_SPEED_LIMITS = true;

    // Relaxed-retry start tolerances.
    private static final double RELAXED_START_POS_M     = 0.02;
    private static final double RELAXED_START_HEAD_RAD  = Math.toRadians(2.0);
    private static final double RELAXED_START_VEL_MPS   = 0.05;
    private static final double RELAXED_START_OMEGA_RPS = Math.toRadians(5.0);

    // ─────────────────────────────────────────────────────────────────────────────
    // Public data types
    // ─────────────────────────────────────────────────────────────────────────────

    public record RobotConfig(
        double massLbs,
        double moi,
        double wheelRadiusInches,
        double wheelCOF,
        double gearRatio,
        double maxMotorRPM,
        double maxMotorTorqueNm,
        Translation2d[] moduleOffsets,
        double maxVelocityOverride,
        double maxAccelOverride
    ) {
        public RobotConfig {
            if (moduleOffsets == null || moduleOffsets.length == 0) {
                throw new IllegalArgumentException("moduleOffsets must be non-empty");
            }
            moduleOffsets = moduleOffsets.clone();
        }

        @Override
        public Translation2d[] moduleOffsets() {
            return moduleOffsets.clone();
        }

        public int moduleCount() {
            return moduleOffsets.length;
        }

        /** Non-copying accessor for hot paths inside this class. */
        Translation2d offset(int i) {
            return moduleOffsets[i];
        }

        public double massKg() {
            return massLbs * 0.45359237;
        }

        public double wheelRadiusMeters() {
            return wheelRadiusInches * 0.0254;
        }

        public double calculateMaxVelocity() {
            if (wheelRadiusMeters() <= 0.0 || gearRatio <= 0.0 || maxMotorRPM <= 0.0) {
                return 0.0;
            }
            double theoretical =
                (maxMotorRPM / 60.0 / gearRatio) * 2.0 * Math.PI * wheelRadiusMeters();
            return Math.min(theoretical, maxVelocityOverride);
        }

        /** Per-module force cap: the tighter of motor torque and available traction share. */
        public double maxForcePerModule() {
            if (wheelRadiusMeters() <= 0.0 || massKg() <= 0.0) {
                return 0.0;
            }
            double motorLimited = maxMotorTorqueNm * gearRatio / wheelRadiusMeters();
            double tractionLimited = wheelCOF * massKg() * 9.80665 / moduleCount();
            return Math.min(motorLimited, tractionLimited);
        }

        public double calculateMaxAcceleration() {
            double force = maxForcePerModule() * moduleCount();
            if (force <= 0.0) {
                return 0.0;
            }
            return Math.min(force / massKg(), maxAccelOverride);
        }

        /** Maximum angular acceleration with every module pushing purely tangentially. */
        public double calculateMaxAngularAcceleration() {
            double radius = getDriveBaseRadius();
            if (radius <= 1e-6 || moi <= 1e-9) {
                return 0.0;
            }
            return maxForcePerModule() * moduleCount() * radius / moi;
        }

        public double getDriveBaseRadius() {
            double radius = 0.0;
            for (Translation2d offset : moduleOffsets) {
                radius = Math.max(radius, offset.getNorm());
            }
            return radius;
        }
    }

    /**
     * Field extents. Hard-coding these was a latent bug; the previous constants
     * (16.54 x 8.21) are the 2023/24 field. Verify the default against the
     * {@code AprilTagFieldLayout} you actually load.
     */
    public record FieldDimensions(double length, double width) {
        public static final FieldDimensions REEFSCAPE_2025 = new FieldDimensions(17.548, 8.052);
        public static final FieldDimensions CRESCENDO_2024 = new FieldDimensions(16.541, 8.211);
    }

    /** A circular obstacle with its own physical radius. */
    public record CircleObstacle(double x, double y, double radius) {
        public CircleObstacle(Translation2d center) {
            this(center.getX(), center.getY(), 0.0);
        }
    }

    /** Axis-aligned rectangle, specified by centre and full extents. */
    public record RectObstacle(double x, double y, double width, double height) {
        public boolean isDegenerate() {
            return width <= 0.0 || height <= 0.0
                || !Double.isFinite(x) || !Double.isFinite(y)
                || !Double.isFinite(width) || !Double.isFinite(height);
        }
    }

    public record ObstacleField(List<CircleObstacle> circles, List<RectObstacle> rects) {
        public static final ObstacleField EMPTY = new ObstacleField(List.of(), List.of());

        public ObstacleField {
            circles = circles == null ? List.of() : List.copyOf(circles);
            rects = rects == null ? List.of() : List.copyOf(rects);
        }
    }

    /**
     * Full planner input. The explicit frame flags exist because a mis-framed
     * {@link ChassisSpeeds} silently corrupts the initial condition, and the old API gave
     * the caller no way to say which frame it was handing over.
     */
    public record PlanRequest(
        Pose2d startPose,
        ChassisSpeeds startSpeeds,
        boolean startSpeedsAreFieldRelative,
        ChassisSpeeds startAccel,
        boolean startAccelIsFieldRelative,
        Pose2d goalPose,
        List<Pose2d> waypoints,
        ObstacleField obstacles,
        double robotBumperRadiusMeters,
        double additionalSafeBufferMeters,
        Trajectory<SwerveSample> previousTrajectory,
        FieldDimensions field
    ) {
        public PlanRequest {
            waypoints = waypoints == null ? List.of() : List.copyOf(waypoints);
            obstacles = obstacles == null ? ObstacleField.EMPTY : obstacles;
            field = field == null ? FieldDimensions.REEFSCAPE_2025 : field;
        }

        /** Minimal request: robot-relative measured speeds, no waypoints, no warm start. */
        public static PlanRequest of(
            Pose2d start, ChassisSpeeds robotRelativeSpeeds, Pose2d goal,
            ObstacleField obstacles, double bumperRadius, double buffer
        ) {
            return new PlanRequest(
                start, robotRelativeSpeeds, false, new ChassisSpeeds(), false,
                goal, List.of(), obstacles, bumperRadius, buffer, null,
                FieldDimensions.REEFSCAPE_2025
            );
        }
    }

    public enum Outcome {
        /** NLP converged and the result passed independent validation. */
        OPTIMIZED,
        /** NLP did not produce a validated result; a validated geometric profile is returned. */
        GEOMETRIC_FALLBACK,
        /** Nothing safe could be produced. The returned trajectory holds the current pose. */
        STATIONARY
    }

    public record PlanResult(List<TrajectoryPoint> points, Outcome outcome, String diagnostics) {
        public boolean isUsable() {
            return outcome != Outcome.STATIONARY;
        }

        public double totalTime() {
            return points.isEmpty() ? 0.0 : points.get(points.size() - 1).time();
        }
    }

    public record TrajectoryPoint(
        double time,
        double x,
        double y,
        double theta,
        double vx,
        double vy,
        double omega,
        double ax,
        double ay,
        double alpha,
        double[] moduleForcesX,
        double[] moduleForcesY
    ) {
        public TrajectoryPoint {
            moduleForcesX = moduleForcesX == null ? new double[0] : moduleForcesX.clone();
            moduleForcesY = moduleForcesY == null ? new double[0] : moduleForcesY.clone();
        }

        @Override
        public double[] moduleForcesX() {
            return moduleForcesX.clone();
        }

        @Override
        public double[] moduleForcesY() {
            return moduleForcesY.clone();
        }

        public Pose2d pose() {
            return new Pose2d(x, y, new Rotation2d(theta));
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Internal geometry types
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * A single smooth, closed, solid keep-out region already inflated by the planning
     * radius. Deliberately <i>not</i> a hollow ring of circles: a discretely sampled NLP
     * can place a knot inside a hollow shell, and the warm start starts near obstacle
     * interiors after shunting.
     */
    private sealed interface KeepOut {
        /** {@code >= 1} outside, {@code < 1} inside. Smooth everywhere except the centre. */
        double level(double x, double y);

        /** Adds "outside this region" to the problem at the given expression pair. */
        void constrain(Problem problem, Variable x, Variable y);

        /** Nearest point outside, pushed out by {@code clearance}. Region is star-shaped
         *  about its centre, so this has a closed form. */
        Translation2d pushOut(double x, double y, double clearance);

        /** Axis-aligned box that fully contains the region. */
        KeepOutBox boundingBox();
    }

    private record CircleKeepOut(double cx, double cy, double radius) implements KeepOut {
        @Override
        public double level(double x, double y) {
            double dx = x - cx;
            double dy = y - cy;
            return (dx * dx + dy * dy) / (radius * radius);
        }

        @Override
        public void constrain(Problem problem, Variable x, Variable y) {
            Variable dx = x.minus(cx);
            Variable dy = y.minus(cy);
            problem.subjectTo(ge(dx.times(dx).plus(dy.times(dy)), radius * radius));
        }

        @Override
        public Translation2d pushOut(double x, double y, double clearance) {
            double dx = x - cx;
            double dy = y - cy;
            double d = Math.hypot(dx, dy);
            if (d < 1e-9) {
                dx = 1.0;
                dy = 0.0;
                d = 1.0;
            }
            double target = radius + clearance;
            return new Translation2d(cx + dx / d * target, cy + dy / d * target);
        }

        @Override
        public KeepOutBox boundingBox() {
            return new KeepOutBox(cx - radius, cx + radius, cy - radius, cy + radius);
        }
    }

    /**
     * {@code (dx/a)^p + (dy/b)^p >= 1} with even {@code p}. Semi-axes are pre-scaled at
     * construction so the region provably contains the true inflated rectangle
     * (the Minkowski sum of the rectangle and the robot disk).
     */
    private record SuperellipseKeepOut(double cx, double cy, double a, double b, int exponent)
        implements KeepOut {

        private int halfExponent() {
            return exponent / 2;
        }

        @Override
        public double level(double x, double y) {
            double ux = sq((x - cx) / a);
            double uy = sq((y - cy) / b);
            return ipow(ux, halfExponent()) + ipow(uy, halfExponent());
        }

        @Override
        public void constrain(Problem problem, Variable x, Variable y) {
            Variable nx = x.minus(cx).div(a);
            Variable ny = y.minus(cy).div(b);
            Variable ux = nx.times(nx);
            Variable uy = ny.times(ny);
            problem.subjectTo(ge(ipow(ux, halfExponent()).plus(ipow(uy, halfExponent())), 1.0));
        }

        @Override
        public Translation2d pushOut(double x, double y, double clearance) {
            double dx = x - cx;
            double dy = y - cy;
            if (Math.hypot(dx, dy) < 1e-9) {
                dx = a;
                dy = 0.0;
            }
            double g = level(cx + dx, cy + dy);
            if (g >= 1.0) {
                return new Translation2d(x, y);
            }
            // level(t*d) == t^p * level(d), so the boundary along this ray is at
            // t = level(d)^(-1/p).
            double t = Math.pow(Math.max(g, 1e-12), -1.0 / exponent);
            double bx = dx * t;
            double by = dy * t;
            double norm = Math.hypot(bx, by);
            double scale = norm > 1e-9 ? (norm + clearance) / norm : 1.0;
            return new Translation2d(cx + bx * scale, cy + by * scale);
        }

        @Override
        public KeepOutBox boundingBox() {
            return new KeepOutBox(cx - a, cx + a, cy - b, cy + b);
        }

        private static double sq(double v) {
            return v * v;
        }

        private static double ipow(double v, int n) {
            double out = 1.0;
            for (int i = 0; i < n; i++) {
                out *= v;
            }
            return out;
        }

        private static Variable ipow(Variable v, int n) {
            Variable out = v;
            for (int i = 1; i < n; i++) {
                out = out.times(v);
            }
            return out;
        }
    }

    private record KeepOutBox(double minX, double maxX, double minY, double maxY) {
        boolean contains(Translation2d p) {
            return p.getX() >= minX && p.getX() <= maxX
                && p.getY() >= minY && p.getY() <= maxY;
        }

        KeepOutBox expanded(double m) {
            return new KeepOutBox(minX - m, maxX + m, minY - m, maxY + m);
        }
    }

    /**
     * Result of geometric routing.
     *
     * <p>{@code firstLegUnvalidated} / {@code lastLegUnvalidated} mean the robot is already
     * inside a conservative keep-out box at that end and the escape segment could not be
     * proven clear. Such a route may still seed the NLP (whose output is independently
     * validated) but must never be emitted as a fallback trajectory.
     */
    private record GeometricRoute(
        List<Translation2d> points,
        boolean firstLegUnvalidated,
        boolean lastLegUnvalidated
    ) {
        boolean safeForFallback() {
            return !firstLegUnvalidated && !lastLegUnvalidated;
        }
    }

    private record QueueNode(int index, double distance) {}

    /** A concrete, named validation failure. Replaces the old opaque boolean. */
    private record Violation(String kind, String detail) {
        @Override
        public String toString() {
            return kind + ": " + detail;
        }
    }

    private record SolveAttempt(String name, int knots, double timeGuess, int samples,
                                boolean relaxStart) {}

    // ─────────────────────────────────────────────────────────────────────────────
    // Trapezoidal profile with a non-zero initial speed
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * Distance-parameterized trapezoid from {@code v0} to rest over {@code distance}.
     *
     * <p>The old planner always profiled from zero, so the fallback commanded an instant
     * stop whenever the robot was already moving. This handles the three real cases:
     * accelerate-cruise-decelerate, triangular, and "already too fast to stop in time"
     * (which reports the deceleration it actually needs).
     */
    private static final class MotionProfile {
        private final double d;
        private final double v0;
        private final double a;
        private final double vPeak;
        private final double tAccel;
        private final double tCruise;
        private final double tDecel;
        private final double decel;
        final boolean overspeed;

        MotionProfile(double distance, double v0, double maxV, double maxA) {
            this.d = Math.max(distance, 0.0);
            this.a = Math.max(maxA, 1e-6);
            double clampedV0 = MathUtil.clamp(v0, 0.0, Math.max(maxV, 1e-6));
            this.v0 = clampedV0;

            if (d <= 1e-9) {
                vPeak = clampedV0;
                tAccel = 0.0;
                tCruise = 0.0;
                tDecel = clampedV0 / a;
                decel = a;
                overspeed = clampedV0 > 1e-6;
                return;
            }

            double stopDist = clampedV0 * clampedV0 / (2.0 * a);
            if (stopDist > d) {
                // Cannot stop within the remaining distance at nominal decel.
                vPeak = clampedV0;
                tAccel = 0.0;
                tCruise = 0.0;
                decel = clampedV0 * clampedV0 / (2.0 * d);
                tDecel = clampedV0 / decel;
                overspeed = true;
                return;
            }

            overspeed = false;
            decel = a;
            double triangularPeak = Math.sqrt((2.0 * a * d + clampedV0 * clampedV0) / 2.0);
            if (triangularPeak <= maxV) {
                vPeak = triangularPeak;
                tAccel = (vPeak - clampedV0) / a;
                tCruise = 0.0;
                tDecel = vPeak / a;
            } else {
                vPeak = maxV;
                tAccel = (vPeak - clampedV0) / a;
                tDecel = vPeak / a;
                double accelDist = (vPeak * vPeak - clampedV0 * clampedV0) / (2.0 * a);
                double decelDist = vPeak * vPeak / (2.0 * a);
                tCruise = Math.max(0.0, (d - accelDist - decelDist) / vPeak);
            }
        }

        double duration() {
            return Math.max(tAccel + tCruise + tDecel, 1e-3);
        }

        double speedAt(double t) {
            t = MathUtil.clamp(t, 0.0, duration());
            if (t <= tAccel) {
                return v0 + a * t;
            }
            if (t <= tAccel + tCruise) {
                return vPeak;
            }
            double tEnd = duration() - t;
            return Math.max(0.0, decel * tEnd);
        }

        double distanceAt(double t) {
            t = MathUtil.clamp(t, 0.0, duration());
            if (t <= tAccel) {
                return v0 * t + 0.5 * a * t * t;
            }
            double accelDist = v0 * tAccel + 0.5 * a * tAccel * tAccel;
            if (t <= tAccel + tCruise) {
                return accelDist + vPeak * (t - tAccel);
            }
            double tEnd = duration() - t;
            return Math.min(d, d - 0.5 * decel * tEnd * tEnd);
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Construction
    // ─────────────────────────────────────────────────────────────────────────────

    private final RobotConfig config;

    public TrajoPlotPro(RobotConfig config) {
        if (config == null) {
            throw new IllegalArgumentException("RobotConfig cannot be null");
        }
        this.config = config;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Legacy entry point (source compatible with the previous version)
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * @deprecated Use {@link #plan(PlanRequest)}. This overload cannot express the frame of
     *     the supplied speeds (assumed robot-relative), circular-obstacle radii (assumed 0),
     *     or the field size (assumed {@link FieldDimensions#REEFSCAPE_2025}).
     */
    @Deprecated
    public List<TrajectoryPoint> solve(
        Pose2d rawStartPose,
        ChassisSpeeds rawCurrentSpeeds,
        ChassisSpeeds rawCurrentAccel,
        Pose2d rawGoalPose,
        List<Pose2d> rawWaypoints,
        List<Translation2d> circularObstacles,
        List<RectObstacle> rectObstacles,
        double robotBumperRadiusMeters,
        double additionalSafeBufferMeters,
        Trajectory<SwerveSample> previousTrajectory
    ) {
        List<CircleObstacle> circles = new ArrayList<>();
        if (circularObstacles != null) {
            for (Translation2d c : circularObstacles) {
                if (c != null) {
                    circles.add(new CircleObstacle(c));
                }
            }
        }

        return plan(new PlanRequest(
            rawStartPose,
            rawCurrentSpeeds,
            false,
            rawCurrentAccel,
            false,
            rawGoalPose,
            rawWaypoints,
            new ObstacleField(circles, rectObstacles),
            robotBumperRadiusMeters,
            additionalSafeBufferMeters,
            previousTrajectory,
            FieldDimensions.REEFSCAPE_2025
        )).points();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Main entry point
    // ─────────────────────────────────────────────────────────────────────────────

    public PlanResult plan(PlanRequest request) {
        StringBuilder log = new StringBuilder();
        log.append("=== TrajoPlotPro.plan() @ ").append(LocalDateTime.now()).append(" ===\n");

        if (request == null || request.startPose() == null || request.goalPose() == null) {
            log.append("ERROR: start and goal poses must be non-null.\n");
            Pose2d fallbackPose = request == null || request.startPose() == null
                ? new Pose2d()
                : request.startPose();
            return finish(buildStationaryTrajectory(fallbackPose), Outcome.STATIONARY, log);
        }

        final FieldDimensions field = request.field();
        final ObstacleField obstacleField = sanitizeObstacles(request.obstacles(), log);
        final int numModules = config.moduleCount();

        final double effectiveRadius =
            Math.max(0.0, request.robotBumperRadiusMeters() + request.additionalSafeBufferMeters());
        final double planningRadius = effectiveRadius + NLP_EXTRA_CLEARANCE_M;

        double rawMaxV = config.calculateMaxVelocity();
        double rawMaxA = config.calculateMaxAcceleration();
        double maxForcePerModule = config.maxForcePerModule();

        if (rawMaxV <= 0.0 || rawMaxA <= 0.0
            || !Double.isFinite(maxForcePerModule) || maxForcePerModule <= 0.0) {
            log.append("ERROR: robot limits are invalid (maxV=").append(rawMaxV)
               .append(", maxA=").append(rawMaxA)
               .append(", Fmod=").append(maxForcePerModule).append(").\n");
            return finish(buildStationaryTrajectory(request.startPose()), Outcome.STATIONARY, log);
        }

        final double maxV = rawMaxV * MODEL_SPEED_SAFETY_FACTOR;
        final double maxA = rawMaxA * MODEL_SPEED_SAFETY_FACTOR;
        final double maxOmega =
            maxV / Math.max(config.getDriveBaseRadius(), 0.05) * MODEL_SPEED_SAFETY_FACTOR;
        final double maxAlpha = Math.max(config.calculateMaxAngularAcceleration(), 1e-3);

        // Normalize to field-relative once, here, so nothing downstream has to guess.
        ChassisSpeeds fieldSpeeds = toFieldRelative(
            request.startSpeeds(), request.startSpeedsAreFieldRelative(),
            request.startPose().getRotation());
        ChassisSpeeds fieldAccel = toFieldRelative(
            request.startAccel(), request.startAccelIsFieldRelative(),
            request.startPose().getRotation());

        final ChassisSpeeds startSpeeds = sanitizeSpeeds(
            fieldSpeeds, maxV * SANITIZE_SAFETY_FACTOR, maxOmega * SANITIZE_SAFETY_FACTOR,
            log, "start speed");
        final ChassisSpeeds startAccel = sanitizeSpeeds(
            fieldAccel, maxA * SANITIZE_SAFETY_FACTOR, Double.MAX_VALUE,
            log, "start accel");

        // ── One shared keep-out environment ────────────────────────────────────
        final List<KeepOut> keepOuts = buildKeepOuts(obstacleField, planningRadius);
        log.append("Environment: ").append(keepOuts.size())
           .append(" keep-out primitives (").append(obstacleField.circles().size())
           .append(" circle(s), ").append(obstacleField.rects().size())
           .append(" rectangle(s) as single superellipses).\n");

        // ── Shunt endpoints to provably legal positions ─────────────────────────
        Optional<Pose2d> shuntedStart =
            shuntToLegal(request.startPose(), keepOuts, field, planningRadius, log, "start");
        Optional<Pose2d> shuntedGoal =
            shuntToLegal(request.goalPose(), keepOuts, field, planningRadius, log, "goal");

        if (shuntedStart.isEmpty() || shuntedGoal.isEmpty()) {
            log.append("ERROR: could not find a legal ")
               .append(shuntedStart.isEmpty() ? "start" : "goal")
               .append(" pose outside the keep-out environment. ")
               .append("The obstacle set or safety buffer is over-constrained.\n");
            return finish(buildStationaryTrajectory(request.startPose()), Outcome.STATIONARY, log);
        }

        final Pose2d startPose = shuntedStart.get();
        final Pose2d goalPose = shuntedGoal.get();

        reportShunt(log, "start", request.startPose(), startPose);
        reportShunt(log, "goal", request.goalPose(), goalPose);

        // Independent cross-check: legal w.r.t. keep-outs must imply legal w.r.t. physical
        // geometry. If this ever fires, the two representations have diverged.
        assertShuntConsistency(startPose, obstacleField, effectiveRadius, field, log, "start");
        assertShuntConsistency(goalPose, obstacleField, effectiveRadius, field, log, "goal");

        List<Pose2d> waypoints = new ArrayList<>();
        int droppedWaypoints = 0;
        for (Pose2d wp : request.waypoints()) {
            if (wp == null) {
                continue;
            }
            Optional<Pose2d> legal =
                shuntToLegal(wp, keepOuts, field, planningRadius, log, "waypoint");
            if (legal.isPresent()) {
                waypoints.add(legal.get());
            } else {
                droppedWaypoints++;
            }
        }
        if (droppedWaypoints > 0) {
            log.append("WARNING: dropped ").append(droppedWaypoints)
               .append(" waypoint(s) with no legal position nearby.\n");
        }
        if (!waypoints.isEmpty()) {
            log.append("NOTE: waypoint rotations are ignored; only positions constrain the route. ")
               .append("Heading is interpolated start -> goal.\n");
        }

        // ── Geometric routing: may fail, and failure is reported honestly ────────
        final List<KeepOutBox> boxes = buildGeometryBoxes(keepOuts);
        Optional<GeometricRoute> maybeRoute =
            buildSafeGeometricRoute(startPose, goalPose, waypoints, boxes, field, planningRadius, log);

        if (maybeRoute.isEmpty()) {
            log.append("ERROR: no geometric route exists through the keep-out environment. ")
               .append("Refusing to substitute a straight line.\n");
            return finish(buildStationaryTrajectory(startPose), Outcome.STATIONARY, log);
        }

        final GeometricRoute route = maybeRoute.get();
        final List<Translation2d> geometricPath = route.points();
        final double totalDist = pathLength(geometricPath);

        Optional<Violation> routeViolation =
            validateGeometricPath(geometricPath, obstacleField, effectiveRadius, field);
        if (routeViolation.isPresent()) {
            log.append("WARNING: geometric route failed independent validation -> ")
               .append(routeViolation.get()).append("\n");
        }

        log.append("Geometric route: ").append(geometricPath.size()).append(" point(s), ")
           .append(fmt(totalDist)).append(" m")
           .append(route.safeForFallback() ? "" : " [endpoint leg unvalidated]")
           .append(".\n");

        // ── Trivial case ────────────────────────────────────────────────────────
        final double startTheta = startPose.getRotation().getRadians();
        // Carried as an unwrapped double: Rotation2d renormalizes and would lose the
        // intended rotation direction for large turns.
        final double goalTheta = startTheta
            + MathUtil.angleModulus(goalPose.getRotation().getRadians() - startTheta);
        final double headingDelta = Math.abs(goalTheta - startTheta);
        final double startSpeed = Math.hypot(startSpeeds.vxMetersPerSecond,
                                             startSpeeds.vyMetersPerSecond);

        if (totalDist < 0.02 && headingDelta < Math.toRadians(1.0) && startSpeed < 0.02) {
            log.append("Already at goal; returning stationary trajectory.\n");
            return finish(buildStationaryTrajectory(goalPose), Outcome.OPTIMIZED, log);
        }

        // ── Sizing: rotation now participates, so pure spins are discretized sanely ──
        MotionProfile translation = new MotionProfile(totalDist, startSpeed, maxV, maxA);
        MotionProfile rotation = new MotionProfile(
            headingDelta, Math.abs(startSpeeds.omegaRadiansPerSecond), maxOmega, maxAlpha);
        final double estT = Math.max(0.25, Math.max(translation.duration(), rotation.duration()));
        final int knots = chooseKnotCount(totalDist, headingDelta, estT);

        if (translation.overspeed) {
            log.append("WARNING: initial speed exceeds what can be stopped within the route ")
               .append("length; the profile requires elevated deceleration.\n");
        }

        log.append("NLP setup: N=").append(knots)
           .append(", estT=").append(fmt(estT)).append(" s")
           .append(", maxV=").append(fmt(maxV))
           .append(", maxA=").append(fmt(maxA))
           .append(", maxOmega=").append(fmt(maxOmega))
           .append(", maxAlpha=").append(fmt(maxAlpha))
           .append(", Fmod=").append(fmt(maxForcePerModule)).append(" N.\n");

        // ── Attempt ladder: cheap and strict first, dense only as a last resort ──
        int denseKnots = Math.min(MAX_KNOT_INTERVALS,
            Math.max(knots + 12, (int) Math.ceil(totalDist / 0.07)));
        List<SolveAttempt> attempts = List.of(
            new SolveAttempt("strict",  knots,      estT, NORMAL_OBSTACLE_SUBDIVISIONS, false),
            new SolveAttempt("relaxed", knots,      estT, NORMAL_OBSTACLE_SUBDIVISIONS, true),
            new SolveAttempt("dense",   denseKnots,
                Math.max(estT, totalDist / Math.max(maxV * 0.85, 0.1)),
                DENSE_OBSTACLE_SUBDIVISIONS, true)
        );

        for (SolveAttempt attempt : attempts) {
            long startNanos = System.nanoTime();
            Optional<List<TrajectoryPoint>> solved = attemptSolve(
                attempt, startPose, startSpeeds, startAccel, goalPose, goalTheta,
                geometricPath, keepOuts, obstacleField, field,
                effectiveRadius, planningRadius,
                maxV, maxOmega, maxForcePerModule, numModules,
                translation, request.previousTrajectory(), log);
            log.append("  attempt '").append(attempt.name()).append("' took ")
               .append(fmt((System.nanoTime() - startNanos) / 1e6)).append(" ms.\n");

            if (solved.isPresent()) {
                return finish(solved.get(), Outcome.OPTIMIZED, log);
            }
        }

        // ── Fallback: only from a fully validated geometric route ───────────────
        if (!route.safeForFallback()) {
            log.append("NLP failed and the geometric route has an unvalidated endpoint leg; ")
               .append("refusing to emit it as a fallback.\n");
            return finish(buildStationaryTrajectory(startPose), Outcome.STATIONARY, log);
        }

        log.append("NLP produced no validated trajectory; using geometric/trapezoidal fallback.\n");
        List<TrajectoryPoint> fallback = buildFallbackTrajectory(
            geometricPath, startPose, goalTheta, startSpeeds, maxV, maxA, numModules);

        Optional<Violation> fallbackViolation = validateTrajectory(
            fallback, obstacleField, field, effectiveRadius, maxV, maxOmega, log);
        if (fallbackViolation.isPresent()) {
            log.append("WARNING: fallback failed validation -> ").append(fallbackViolation.get())
               .append("; returning stationary trajectory.\n");
            return finish(buildStationaryTrajectory(startPose), Outcome.STATIONARY, log);
        }

        return finish(fallback, Outcome.GEOMETRIC_FALLBACK, log);
    }

    private PlanResult finish(List<TrajectoryPoint> points, Outcome outcome, StringBuilder log) {
        log.append("Result: ").append(outcome).append(", ")
           .append(points.size()).append(" point(s).\n");
        String diagnostics = log.toString();
        writeLog(diagnostics);
        return new PlanResult(List.copyOf(points), outcome, diagnostics);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Environment construction
    // ─────────────────────────────────────────────────────────────────────────────

    private ObstacleField sanitizeObstacles(ObstacleField raw, StringBuilder log) {
        List<CircleObstacle> circles = new ArrayList<>();
        for (CircleObstacle c : raw.circles()) {
            if (c == null || !Double.isFinite(c.x()) || !Double.isFinite(c.y())
                || !Double.isFinite(c.radius()) || c.radius() < 0.0) {
                log.append("WARNING: dropped a malformed circular obstacle.\n");
                continue;
            }
            circles.add(c);
        }

        List<RectObstacle> rects = new ArrayList<>();
        for (RectObstacle r : raw.rects()) {
            if (r == null || r.isDegenerate()) {
                log.append("WARNING: dropped a degenerate rectangular obstacle.\n");
                continue;
            }
            rects.add(r);
        }

        return new ObstacleField(circles, rects);
    }

    /**
     * One keep-out primitive per obstacle. This is the change that removes roughly two
     * orders of magnitude of NLP constraints relative to the circle-grid approach.
     */
    private List<KeepOut> buildKeepOuts(ObstacleField obstacles, double robotRadius) {
        List<KeepOut> keepOuts = new ArrayList<>();

        for (CircleObstacle c : obstacles.circles()) {
            keepOuts.add(new CircleKeepOut(c.x(), c.y(), Math.max(c.radius() + robotRadius, 1e-3)));
        }

        for (RectObstacle rect : obstacles.rects()) {
            keepOuts.add(buildRectKeepOut(rect, robotRadius));
        }

        return keepOuts;
    }

    /**
     * Wraps the inflated rectangle in the smallest superellipse of the configured exponent
     * that provably contains it. {@code kappa} is found by sampling the true boundary
     * (two straight edges plus the corner arc, by symmetry), so the result is conservative
     * by construction rather than by hope.
     */
    private KeepOut buildRectKeepOut(RectObstacle rect, double robotRadius) {
        double a0 = rect.width() * 0.5 + robotRadius;
        double b0 = rect.height() * 0.5 + robotRadius;
        double halfW = rect.width() * 0.5;
        double halfH = rect.height() * 0.5;
        int p = SUPERELLIPSE_EXPONENT;

        double worst = 1.0;
        final int samples = 96;

        for (int i = 0; i <= samples; i++) {
            double s = (double) i / samples;
            // Top edge of the Minkowski sum: y = halfH + r, x in [0, halfW].
            worst = Math.max(worst, levelRaw(halfW * s, halfH + robotRadius, a0, b0, p));
            // Right edge: x = halfW + r, y in [0, halfH].
            worst = Math.max(worst, levelRaw(halfW + robotRadius, halfH * s, a0, b0, p));
            // Corner quarter-arc.
            double phi = s * Math.PI / 2.0;
            worst = Math.max(worst,
                levelRaw(halfW + robotRadius * Math.cos(phi),
                         halfH + robotRadius * Math.sin(phi), a0, b0, p));
        }

        double kappa = Math.pow(worst, 1.0 / p) * (1.0 + 1e-3);
        return new SuperellipseKeepOut(rect.x(), rect.y(), a0 * kappa, b0 * kappa, p);
    }

    private static double levelRaw(double dx, double dy, double a, double b, int p) {
        double ux = (dx / a) * (dx / a);
        double uy = (dy / b) * (dy / b);
        double out = 1.0;
        double vx = 1.0;
        double vy = 1.0;
        for (int i = 0; i < p / 2; i++) {
            vx *= ux;
            vy *= uy;
        }
        out = vx + vy;
        return out;
    }

    private List<KeepOutBox> buildGeometryBoxes(List<KeepOut> keepOuts) {
        List<KeepOutBox> boxes = new ArrayList<>(keepOuts.size());
        for (KeepOut keepOut : keepOuts) {
            boxes.add(keepOut.boundingBox().expanded(GEOMETRIC_CLEARANCE_M));
        }
        return boxes;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Shunting: guaranteed legal or explicit failure
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * Projects a pose out of every keep-out and inside the field, then <b>verifies</b> the
     * result. The old version ran a fixed iteration budget and returned whatever it had,
     * which could still be illegal when inflated regions overlap. If projection does not
     * converge, a spiral search finds the nearest legal point; if that also fails, the
     * caller is told, rather than being handed an illegal pose.
     */
    private Optional<Pose2d> shuntToLegal(
        Pose2d pose, List<KeepOut> keepOuts, FieldDimensions field,
        double margin, StringBuilder log, String label
    ) {
        double px = pose.getX();
        double py = pose.getY();

        for (int iter = 0; iter < 60; iter++) {
            boolean moved = false;

            Translation2d clamped = clampToField(new Translation2d(px, py), field, margin);
            if (Math.abs(clamped.getX() - px) > 1e-9 || Math.abs(clamped.getY() - py) > 1e-9) {
                px = clamped.getX();
                py = clamped.getY();
                moved = true;
            }

            for (KeepOut keepOut : keepOuts) {
                if (keepOut.level(px, py) < 1.0) {
                    Translation2d pushed = keepOut.pushOut(px, py, SHUNT_CLEARANCE_M);
                    px = pushed.getX();
                    py = pushed.getY();
                    moved = true;
                }
            }

            if (!moved) {
                break;
            }
        }

        if (isLegal(px, py, keepOuts, field, margin)) {
            return Optional.of(new Pose2d(px, py, pose.getRotation()));
        }

        log.append("NOTE: iterative projection of the ").append(label)
           .append(" pose did not converge (overlapping keep-outs); running spiral search.\n");

        Optional<Translation2d> found = spiralSearch(
            new Translation2d(pose.getX(), pose.getY()),
            p -> isLegal(p.getX(), p.getY(), keepOuts, field, margin));

        return found.map(p -> new Pose2d(p, pose.getRotation()));
    }

    private boolean isLegal(double x, double y, List<KeepOut> keepOuts,
                            FieldDimensions field, double margin) {
        if (!isInsideSafeField(new Translation2d(x, y), field, margin)) {
            return false;
        }
        for (KeepOut keepOut : keepOuts) {
            if (keepOut.level(x, y) < 1.0) {
                return false;
            }
        }
        return true;
    }

    /** Expanding ring sampler: returns the nearest accepted point, or empty. */
    private Optional<Translation2d> spiralSearch(
        Translation2d origin, java.util.function.Predicate<Translation2d> accept
    ) {
        if (accept.test(origin)) {
            return Optional.of(origin);
        }
        for (double radius = 0.05; radius <= 3.0 + 1e-9; radius += 0.05) {
            int spokes = Math.max(12, (int) Math.ceil(2.0 * Math.PI * radius / 0.04));
            for (int i = 0; i < spokes; i++) {
                double angle = 2.0 * Math.PI * i / spokes;
                Translation2d candidate = new Translation2d(
                    origin.getX() + radius * Math.cos(angle),
                    origin.getY() + radius * Math.sin(angle));
                if (accept.test(candidate)) {
                    return Optional.of(candidate);
                }
            }
        }
        return Optional.empty();
    }

    private void reportShunt(StringBuilder log, String label, Pose2d before, Pose2d after) {
        double moved = before.getTranslation().getDistance(after.getTranslation());
        if (moved > 1e-3) {
            log.append("NOTE: ").append(label).append(" pose moved ").append(fmt(moved))
               .append(" m to clear an obstacle or the field boundary.\n");
        }
    }

    private void assertShuntConsistency(
        Pose2d pose, ObstacleField obstacles, double effectiveRadius,
        FieldDimensions field, StringBuilder log, String label
    ) {
        Optional<Violation> violation = describeObstacleViolation(
            pose.getX(), pose.getY(), obstacles, effectiveRadius);
        if (violation.isPresent()) {
            log.append("BUG: sanitized ").append(label)
               .append(" pose is legal w.r.t. keep-outs but violates physical geometry -> ")
               .append(violation.get())
               .append(" (the two representations have diverged).\n");
        }
        if (!isPointWithinField(pose.getX(), pose.getY(), field, effectiveRadius)) {
            log.append("BUG: sanitized ").append(label).append(" pose is outside the field.\n");
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Geometric routing
    // ─────────────────────────────────────────────────────────────────────────────

    private Optional<GeometricRoute> buildSafeGeometricRoute(
        Pose2d start, Pose2d goal, List<Pose2d> waypoints, List<KeepOutBox> boxes,
        FieldDimensions field, double margin, StringBuilder log
    ) {
        List<Translation2d> anchors = new ArrayList<>();
        anchors.add(start.getTranslation());
        for (Pose2d wp : waypoints) {
            anchors.add(wp.getTranslation());
        }
        anchors.add(goal.getTranslation());

        List<Translation2d> route = new ArrayList<>();
        route.add(anchors.get(0));
        boolean firstLegUnvalidated = false;
        boolean lastLegUnvalidated = false;

        for (int i = 0; i < anchors.size() - 1; i++) {
            Translation2d from = anchors.get(i);
            Translation2d to = anchors.get(i + 1);

            LegResult leg = routeLeg(from, to, boxes, field, margin);
            if (leg == null) {
                log.append("ERROR: no route for leg ").append(i)
                   .append(" (").append(fmtPoint(from)).append(" -> ")
                   .append(fmtPoint(to)).append(").\n");
                return Optional.empty();
            }

            if (leg.startedInside() && i == 0) {
                firstLegUnvalidated = true;
                log.append("NOTE: start is inside a conservative keep-out box; ")
                   .append("the first leg is an escape segment and is not certified clear.\n");
            }
            if (leg.endedInside() && i == anchors.size() - 2) {
                lastLegUnvalidated = true;
                log.append("NOTE: goal is inside a conservative keep-out box; ")
                   .append("the last leg is an approach segment and is not certified clear.\n");
            }
            if (leg.startedInside() && i > 0) {
                firstLegUnvalidated = true;
            }
            if (leg.endedInside() && i < anchors.size() - 2) {
                lastLegUnvalidated = true;
            }

            for (int j = 1; j < leg.points().size(); j++) {
                Translation2d next = leg.points().get(j);
                if (route.get(route.size() - 1).getDistance(next) > 1e-6) {
                    route.add(next);
                }
            }
        }

        if (route.size() < 2) {
            route.add(anchors.get(anchors.size() - 1));
        }

        return Optional.of(new GeometricRoute(
            simplifyCollinearPath(route), firstLegUnvalidated, lastLegUnvalidated));
    }

    private record LegResult(List<Translation2d> points, boolean startedInside, boolean endedInside) {}

    /**
     * Routes one leg. An endpoint inside a keep-out box is handled as a special case — it
     * escapes to free space and keeps routing — rather than abandoning obstacle avoidance,
     * which was the previous version's central bug.
     */
    private LegResult routeLeg(
        Translation2d from, Translation2d to, List<KeepOutBox> boxes,
        FieldDimensions field, double margin
    ) {
        boolean fromInside = pointInsideAnyBox(from, boxes);
        boolean toInside = pointInsideAnyBox(to, boxes);

        Translation2d routeFrom = from;
        Translation2d routeTo = to;

        if (fromInside) {
            Optional<Translation2d> escape = spiralSearch(from,
                p -> !pointInsideAnyBox(p, boxes) && isInsideSafeField(p, field, margin));
            if (escape.isEmpty()) {
                return null;
            }
            routeFrom = escape.get();
        }

        if (toInside) {
            Optional<Translation2d> escape = spiralSearch(to,
                p -> !pointInsideAnyBox(p, boxes) && isInsideSafeField(p, field, margin));
            if (escape.isEmpty()) {
                return null;
            }
            routeTo = escape.get();
        }

        Optional<List<Translation2d>> core =
            findVisibilityPath(routeFrom, routeTo, boxes, field, margin);
        if (core.isEmpty()) {
            return null;
        }

        List<Translation2d> points = new ArrayList<>();
        if (fromInside) {
            points.add(from);
        }
        points.addAll(core.get());
        if (toInside) {
            points.add(to);
        }

        return new LegResult(points, fromInside, toInside);
    }

    /**
     * Dijkstra over a corner visibility graph.
     *
     * @return the route, or {@link Optional#empty()} if none exists. It <b>never</b>
     *     degrades to {@code List.of(start, goal)}: that loophole is what let the planner
     *     drive through a rectangle after a routing failure.
     */
    private Optional<List<Translation2d>> findVisibilityPath(
        Translation2d start, Translation2d goal, List<KeepOutBox> boxes,
        FieldDimensions field, double margin
    ) {
        if (pointInsideAnyBox(start, boxes) || pointInsideAnyBox(goal, boxes)) {
            return Optional.empty();
        }
        if (segmentClearOfBoxes(start, goal, boxes, field, margin)) {
            return Optional.of(List.of(start, goal));
        }

        List<Translation2d> nodes = new ArrayList<>();
        nodes.add(start);
        nodes.add(goal);

        final double cornerOffset = 0.02;
        for (KeepOutBox box : boxes) {
            Translation2d[] corners = {
                new Translation2d(box.minX() - cornerOffset, box.minY() - cornerOffset),
                new Translation2d(box.minX() - cornerOffset, box.maxY() + cornerOffset),
                new Translation2d(box.maxX() + cornerOffset, box.minY() - cornerOffset),
                new Translation2d(box.maxX() + cornerOffset, box.maxY() + cornerOffset)
            };
            for (Translation2d corner : corners) {
                if (isInsideSafeField(corner, field, margin) && !pointInsideAnyBox(corner, boxes)) {
                    nodes.add(corner);
                }
            }
        }

        int n = nodes.size();
        double[] dist = new double[n];
        int[] prev = new int[n];
        boolean[] visited = new boolean[n];
        java.util.Arrays.fill(dist, Double.POSITIVE_INFINITY);
        java.util.Arrays.fill(prev, -1);
        dist[0] = 0.0;

        PriorityQueue<QueueNode> queue =
            new PriorityQueue<>(Comparator.comparingDouble(QueueNode::distance));
        queue.add(new QueueNode(0, 0.0));

        while (!queue.isEmpty()) {
            QueueNode current = queue.poll();
            int i = current.index();
            if (visited[i]) {
                continue;
            }
            visited[i] = true;
            if (i == 1) {
                break;
            }

            for (int j = 0; j < n; j++) {
                if (i == j || visited[j]) {
                    continue;
                }
                if (!segmentClearOfBoxes(nodes.get(i), nodes.get(j), boxes, field, margin)) {
                    continue;
                }
                double candidate = dist[i] + nodes.get(i).getDistance(nodes.get(j));
                if (candidate < dist[j] - 1e-12) {
                    dist[j] = candidate;
                    prev[j] = i;
                    queue.add(new QueueNode(j, candidate));
                }
            }
        }

        if (!Double.isFinite(dist[1])) {
            return Optional.empty();
        }

        List<Translation2d> path = new ArrayList<>();
        for (int at = 1; at >= 0; at = prev[at]) {
            path.add(nodes.get(at));
            if (at == 0) {
                break;
            }
        }
        Collections.reverse(path);
        return Optional.of(path);
    }

    private boolean isInsideSafeField(Translation2d p, FieldDimensions field, double robotRadius) {
        double m = robotRadius + FIELD_EDGE_EXTRA_MARGIN_M;
        return p.getX() >= m && p.getX() <= field.length() - m
            && p.getY() >= m && p.getY() <= field.width() - m;
    }

    private Translation2d clampToField(Translation2d p, FieldDimensions field, double robotRadius) {
        double m = robotRadius + FIELD_EDGE_EXTRA_MARGIN_M;
        return new Translation2d(
            MathUtil.clamp(p.getX(), m, Math.max(m, field.length() - m)),
            MathUtil.clamp(p.getY(), m, Math.max(m, field.width() - m)));
    }

    private boolean pointInsideAnyBox(Translation2d p, List<KeepOutBox> boxes) {
        for (KeepOutBox box : boxes) {
            if (box.contains(p)) {
                return true;
            }
        }
        return false;
    }

    private boolean segmentClearOfBoxes(
        Translation2d a, Translation2d b, List<KeepOutBox> boxes,
        FieldDimensions field, double margin
    ) {
        if (!isInsideSafeField(a, field, margin) || !isInsideSafeField(b, field, margin)) {
            return false;
        }
        for (KeepOutBox box : boxes) {
            if (segmentIntersectsBox(a, b, box)) {
                return false;
            }
        }
        return true;
    }

    /** Liang-Barsky slab test, restricted to t in [0, 1]. */
    private boolean segmentIntersectsBox(Translation2d a, Translation2d b, KeepOutBox box) {
        double[] p = {a.getX(), a.getY()};
        double[] d = {b.getX() - a.getX(), b.getY() - a.getY()};
        double[] min = {box.minX(), box.minY()};
        double[] max = {box.maxX(), box.maxY()};

        double tMin = 0.0;
        double tMax = 1.0;

        for (int axis = 0; axis < 2; axis++) {
            if (Math.abs(d[axis]) < 1e-12) {
                if (p[axis] < min[axis] || p[axis] > max[axis]) {
                    return false;
                }
                continue;
            }
            double t1 = (min[axis] - p[axis]) / d[axis];
            double t2 = (max[axis] - p[axis]) / d[axis];
            if (t1 > t2) {
                double tmp = t1;
                t1 = t2;
                t2 = tmp;
            }
            tMin = Math.max(tMin, t1);
            tMax = Math.min(tMax, t2);
            if (tMin > tMax) {
                return false;
            }
        }
        return true;
    }

    /**
     * Drops interior vertices that are nearly collinear. Uses perpendicular <i>distance</i>
     * rather than the raw cross product, which had units of m^2 and therefore a
     * scale-dependent threshold.
     */
    private List<Translation2d> simplifyCollinearPath(List<Translation2d> path) {
        if (path.size() <= 2) {
            return List.copyOf(path);
        }

        final double toleranceM = 1e-3;
        List<Translation2d> out = new ArrayList<>();
        out.add(path.get(0));

        for (int i = 1; i < path.size() - 1; i++) {
            Translation2d a = out.get(out.size() - 1);
            Translation2d b = path.get(i);
            Translation2d c = path.get(i + 1);

            double abx = b.getX() - a.getX();
            double aby = b.getY() - a.getY();
            double acx = c.getX() - a.getX();
            double acy = c.getY() - a.getY();
            double baseLength = Math.hypot(acx, acy);

            if (baseLength < 1e-9) {
                continue;
            }
            double perpendicular = Math.abs(abx * acy - aby * acx) / baseLength;
            if (perpendicular > toleranceM) {
                out.add(b);
            }
        }

        out.add(path.get(path.size() - 1));
        return out;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Discretization
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * Rotation now participates in sizing. Previously a near-in-place 180-degree turn was
     * sized purely by translation distance, yielding far too few knots and a warm start
     * whose heading ramp the dynamics could not reproduce.
     */
    private int chooseKnotCount(double totalDist, double headingDelta, double estT) {
        int byDistance = (int) Math.ceil(totalDist / MAX_KNOT_SPACING_M);
        int byRotation = (int) Math.ceil(headingDelta / Math.toRadians(12.0));
        int byTime = (int) Math.ceil(estT / MAX_DT_S);
        int desired = Math.max(byTime, Math.max(byDistance, byRotation));
        return Math.max(MIN_KNOT_INTERVALS, Math.min(MAX_KNOT_INTERVALS, desired));
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // NLP
    // ─────────────────────────────────────────────────────────────────────────────

    private Optional<List<TrajectoryPoint>> attemptSolve(
        SolveAttempt attempt,
        Pose2d startPose,
        ChassisSpeeds startSpeedsField,
        ChassisSpeeds startAccelField,
        Pose2d goalPose,
        double goalTheta,
        List<Translation2d> geometricPath,
        List<KeepOut> keepOuts,
        ObstacleField obstacles,
        FieldDimensions field,
        double effectiveRadius,
        double planningRadius,
        double maxV,
        double maxOmega,
        double maxForcePerModule,
        int numModules,
        MotionProfile translationProfile,
        Trajectory<SwerveSample> previousTrajectory,
        StringBuilder log
    ) {
        final int n = attempt.knots();

        try (Problem problem = new Problem()) {
            VariableMatrix x = problem.decisionVariable(6, n + 1);
            VariableMatrix u = problem.decisionVariable(2 * numModules, n);
            Variable totalTime = problem.decisionVariable();

            problem.subjectTo(bounds(0.05, totalTime, 60.0));
            Variable dt = totalTime.div((double) n);

            // ── Dynamics, module force limits, module speed limits ──────────────
            for (int k = 0; k < n; k++) {
                Variable theta = x.get(2, k);
                Variable cosT = cos(theta);
                Variable sinT = sin(theta);

                Variable netFx = new Variable(0.0);
                Variable netFy = new Variable(0.0);
                Variable netTau = new Variable(0.0);

                for (int i = 0; i < numModules; i++) {
                    Variable fx = u.get(i, k);
                    Variable fy = u.get(i + numModules, k);

                    // Friction/torque disk, not a box: a box would permit sqrt(2)x on the
                    // diagonal.
                    problem.subjectTo(le(fx.times(fx).plus(fy.times(fy)),
                        maxForcePerModule * maxForcePerModule));

                    Translation2d off = config.offset(i);
                    Variable rx = cosT.times(off.getX()).minus(sinT.times(off.getY()));
                    Variable ry = sinT.times(off.getX()).plus(cosT.times(off.getY()));

                    netFx = netFx.plus(fx);
                    netFy = netFy.plus(fy);
                    netTau = netTau.plus(rx.times(fy).minus(ry.times(fx)));
                }

                Variable ax = netFx.div(config.massKg());
                Variable ay = netFy.div(config.massKg());
                Variable alpha = netTau.div(config.moi());

                // Semi-implicit (symplectic) Euler: velocity first, then position with the
                // updated velocity. Position is therefore exactly linear in t across the
                // interval, which is what makes sparse obstacle sampling defensible.
                Variable vxNext = x.get(3, k).plus(ax.times(dt));
                Variable vyNext = x.get(4, k).plus(ay.times(dt));
                Variable omegaNext = x.get(5, k).plus(alpha.times(dt));

                problem.subjectTo(eq(x.get(3, k + 1), vxNext));
                problem.subjectTo(eq(x.get(4, k + 1), vyNext));
                problem.subjectTo(eq(x.get(5, k + 1), omegaNext));
                problem.subjectTo(eq(x.get(0, k + 1), x.get(0, k).plus(vxNext.times(dt))));
                problem.subjectTo(eq(x.get(1, k + 1), x.get(1, k).plus(vyNext.times(dt))));
                problem.subjectTo(eq(x.get(2, k + 1), theta.plus(omegaNext.times(dt))));
            }

            // Field bounds use the planning radius, matching the shunt and the router.
            // The old code bounded the NLP by effectiveRadius, making the optimizer
            // strictly less strict than its own sanitizer.
            problem.subjectTo(bounds(planningRadius, x.row(0), field.length() - planningRadius));
            problem.subjectTo(bounds(planningRadius, x.row(1), field.width() - planningRadius));
            problem.subjectTo(bounds(-maxOmega, x.row(5), maxOmega));

            for (int k = 0; k <= n; k++) {
                Variable vx = x.get(3, k);
                Variable vy = x.get(4, k);
                problem.subjectTo(le(vx.times(vx).plus(vy.times(vy)), maxV * maxV));

                // Chassis speed alone is not the real limit: a module's wheel saturates on
                // translation + rotation combined. Without this the solver can request
                // module speeds above free speed and the follower silently desaturates,
                // which destroys path tracking.
                if (ENFORCE_MODULE_SPEED_LIMITS) {
                    Variable cosT = cos(x.get(2, k));
                    Variable sinT = sin(x.get(2, k));
                    Variable omega = x.get(5, k);

                    for (int i = 0; i < numModules; i++) {
                        Translation2d off = config.offset(i);
                        Variable rx = cosT.times(off.getX()).minus(sinT.times(off.getY()));
                        Variable ry = sinT.times(off.getX()).plus(cosT.times(off.getY()));
                        Variable mvx = vx.minus(omega.times(ry));
                        Variable mvy = vy.plus(omega.times(rx));
                        problem.subjectTo(
                            le(mvx.times(mvx).plus(mvy.times(mvy)), maxV * maxV));
                    }
                }
            }

            // ── Obstacle avoidance ──────────────────────────────────────────────
            // One primitive per obstacle, sampled at interval fractions. Constraint count
            // is keepOuts * (n * samples + 1) instead of (circlesPerRect * rects) * n * 3.
            int obstacleConstraints = 0;
            for (KeepOut keepOut : keepOuts) {
                for (int k = 0; k < n; k++) {
                    for (int s = 0; s < attempt.samples(); s++) {
                        double fraction = (double) s / attempt.samples();
                        constrainOutside(problem, x, k, fraction, keepOut);
                        obstacleConstraints++;
                    }
                }
                constrainOutside(problem, x, n - 1, 1.0, keepOut);
                obstacleConstraints++;
            }

            // ── Boundary conditions ─────────────────────────────────────────────
            boolean relax = attempt.relaxStart();
            pinState(problem, x, 0, startPose.getTranslation(),
                startPose.getRotation().getRadians(), startSpeedsField,
                relax ? RELAXED_START_POS_M : 0.0,
                relax ? RELAXED_START_HEAD_RAD : 0.0,
                relax ? RELAXED_START_VEL_MPS : 0.0,
                relax ? RELAXED_START_OMEGA_RPS : 0.0);

            pinState(problem, x, n, goalPose.getTranslation(), goalTheta,
                new ChassisSpeeds(), 0.0, 0.0, 0.0, 0.0);

            // ── Objective ───────────────────────────────────────────────────────
            // Normalized and averaged so the regularizer is a genuine tie-breaker. The old
            // `T + 1e-6 * sum(u^2)` summed to the same order of magnitude as T for typical
            // forces, so it was quietly a minimum-effort problem, not a minimum-time one.
            Variable cost = totalTime;
            double effortScale = EFFORT_WEIGHT
                / (maxForcePerModule * maxForcePerModule * n * 2.0 * numModules);
            for (int k = 0; k < n; k++) {
                for (int i = 0; i < 2 * numModules; i++) {
                    Variable force = u.get(i, k);
                    cost = cost.plus(force.times(force).times(effortScale));
                }
            }
            problem.minimize(cost);

            applyWarmStart(x, u, totalTime, startPose, startSpeedsField, startAccelField,
                goalTheta, geometricPath, translationProfile, maxV, maxForcePerModule,
                n, numModules, previousTrajectory, log);

            log.append("  '").append(attempt.name()).append("': N=").append(n)
               .append(", samples/interval=").append(attempt.samples())
               .append(", obstacle constraints=").append(obstacleConstraints)
               .append(", vars=").append(6 * (n + 1) + 2 * numModules * n + 1).append("\n");

            // TODO: cap iterations / wall time here once the Sleipnir build in use exposes
            // solver options. An unbounded solve on the RIO is a real hazard; until then,
            // call plan() from a background executor and time-box it on the consumer side.
            ExitStatus status = problem.solve();

            log.append("  solver exit (").append(attempt.name()).append("): ")
               .append(status).append("\n");

            if (!ExitStatus.SUCCESS.equals(status)) {
                return Optional.empty();
            }

            List<TrajectoryPoint> result = extractSolution(totalTime, x, u, n, numModules);

            // Independent of the NLP by construction: raw physical geometry, denser
            // sampling, real field extents. A numerical SUCCESS is not a safety argument.
            Optional<Violation> violation = validateTrajectory(
                result, obstacles, field, effectiveRadius, maxV, maxOmega, log);
            if (violation.isPresent()) {
                log.append("  SUCCESS but rejected by post-solve validation -> ")
                   .append(violation.get()).append("\n");
                return Optional.empty();
            }

            return Optional.of(result);

        } catch (RuntimeException e) {
            log.append("  exception during '").append(attempt.name()).append("': ")
               .append(e).append("\n");
            return Optional.empty();
        }
    }

    private void constrainOutside(
        Problem problem, VariableMatrix x, int k, double fraction, KeepOut keepOut
    ) {
        Variable px;
        Variable py;

        if (fraction <= 0.0) {
            px = x.get(0, k);
            py = x.get(1, k);
        } else if (fraction >= 1.0) {
            px = x.get(0, k + 1);
            py = x.get(1, k + 1);
        } else {
            px = x.get(0, k).plus(x.get(0, k + 1).minus(x.get(0, k)).times(fraction));
            py = x.get(1, k).plus(x.get(1, k + 1).minus(x.get(1, k)).times(fraction));
        }

        keepOut.constrain(problem, px, py);
    }

    /** Pins a knot to a field-relative state, exactly or within a tolerance box. */
    private void pinState(
        Problem problem, VariableMatrix x, int k, Translation2d position, double theta,
        ChassisSpeeds fieldSpeeds, double posTol, double headingTol, double velTol, double omegaTol
    ) {
        pinValue(problem, x.get(0, k), position.getX(), posTol);
        pinValue(problem, x.get(1, k), position.getY(), posTol);
        pinValue(problem, x.get(2, k), theta, headingTol);
        pinValue(problem, x.get(3, k), fieldSpeeds.vxMetersPerSecond, velTol);
        pinValue(problem, x.get(4, k), fieldSpeeds.vyMetersPerSecond, velTol);
        pinValue(problem, x.get(5, k), fieldSpeeds.omegaRadiansPerSecond, omegaTol);
    }

    private void pinValue(Problem problem, Variable variable, double target, double tolerance) {
        if (tolerance <= 0.0) {
            problem.subjectTo(eq(variable, target));
        } else {
            problem.subjectTo(bounds(target - tolerance, variable, target + tolerance));
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Warm start
    // ─────────────────────────────────────────────────────────────────────────────

    private void applyWarmStart(
        VariableMatrix x, VariableMatrix u, Variable totalTime,
        Pose2d start, ChassisSpeeds startSpeedsField, ChassisSpeeds startAccelField,
        double goalTheta, List<Translation2d> geometricPath, MotionProfile profile,
        double maxV, double maxForcePerModule, int n, int numModules,
        Trajectory<SwerveSample> previousTrajectory, StringBuilder log
    ) {
        double estT = Math.max(profile.duration(), 0.25);
        totalTime.setValue(estT);

        double[] vxGuess = new double[n + 1];
        double[] vyGuess = new double[n + 1];
        double[] omegaGuess = new double[n + 1];

        // Prefer the previous solution when one is available and still relevant: it is a
        // dynamically feasible seed, which the geometric guess is not. Falls back silently.
        Optional<double[][]> reused =
            resamplePreviousTrajectory(previousTrajectory, start, n, log);

        double startTheta = start.getRotation().getRadians();
        double totalDist = pathLength(geometricPath);

        for (int k = 0; k <= n; k++) {
            double t = estT * k / n;
            double frac = (double) k / n;

            double px;
            double py;
            double theta;

            if (reused.isPresent()) {
                double[] sample = reused.get()[k];
                px = sample[0];
                py = sample[1];
                theta = sample[2];
                vxGuess[k] = sample[3];
                vyGuess[k] = sample[4];
                omegaGuess[k] = sample[5];
            } else {
                double dist = profile.distanceAt(t);
                Translation2d p = interpolateAtDistance(geometricPath, dist);
                Translation2d ahead = interpolateAtDistance(
                    geometricPath, Math.min(dist + WARM_START_LOOKAHEAD_M, totalDist));

                px = p.getX();
                py = p.getY();
                theta = startTheta + frac * (goalTheta - startTheta);

                double dx = ahead.getX() - p.getX();
                double dy = ahead.getY() - p.getY();
                double norm = Math.hypot(dx, dy);
                double speed = Math.min(profile.speedAt(t), maxV * 0.995);

                if (k == 0) {
                    vxGuess[k] = startSpeedsField.vxMetersPerSecond;
                    vyGuess[k] = startSpeedsField.vyMetersPerSecond;
                } else if (norm > 1e-9 && k < n) {
                    vxGuess[k] = speed * dx / norm;
                    vyGuess[k] = speed * dy / norm;
                } else {
                    vxGuess[k] = 0.0;
                    vyGuess[k] = 0.0;
                }

                // The old guess set omega = 0 at every knot while ramping theta linearly,
                // i.e. a seed that contradicted its own kinematics. Use the ramp's rate.
                if (k == 0) {
                    omegaGuess[k] = startSpeedsField.omegaRadiansPerSecond;
                } else if (k == n) {
                    omegaGuess[k] = 0.0;
                } else {
                    omegaGuess[k] = (goalTheta - startTheta) / estT;
                }
            }

            x.get(0, k).setValue(px);
            x.get(1, k).setValue(py);
            x.get(2, k).setValue(theta);
            x.get(3, k).setValue(vxGuess[k]);
            x.get(4, k).setValue(vyGuess[k]);
            x.get(5, k).setValue(omegaGuess[k]);
        }

        double dt = estT / n;
        for (int k = 0; k < n; k++) {
            double ax;
            double ay;

            if (k == 0) {
                ax = startAccelField.vxMetersPerSecond;
                ay = startAccelField.vyMetersPerSecond;
            } else {
                ax = (vxGuess[k + 1] - vxGuess[k]) / Math.max(dt, 1e-6);
                ay = (vyGuess[k + 1] - vyGuess[k]) / Math.max(dt, 1e-6);
            }

            double perFx = config.massKg() * ax / numModules;
            double perFy = config.massKg() * ay / numModules;
            double magnitude = Math.hypot(perFx, perFy);
            if (magnitude > maxForcePerModule && magnitude > 1e-9) {
                double scale = maxForcePerModule / magnitude;
                perFx *= scale;
                perFy *= scale;
            }

            for (int i = 0; i < numModules; i++) {
                u.get(i, k).setValue(perFx);
                u.get(i + numModules, k).setValue(perFy);
            }
        }
    }

    /**
     * Resamples the previous Choreo trajectory onto {@code n + 1} knots.
     *
     * <p>Isolated here, and defensive against both exceptions and {@link LinkageError}, so a
     * vendordep signature change degrades to "no warm start" instead of breaking the planner.
     * Rejected if the previous trajectory does not start near the current pose, since a stale
     * seed is worse than a geometric one.
     */
    private Optional<double[][]> resamplePreviousTrajectory(
        Trajectory<SwerveSample> previous, Pose2d start, int n, StringBuilder log
    ) {
        if (previous == null) {
            return Optional.empty();
        }

        try {
            List<SwerveSample> samples = previous.samples();
            if (samples == null || samples.size() < 2) {
                return Optional.empty();
            }

            SwerveSample first = samples.get(0);
            double offset = Math.hypot(first.x - start.getX(), first.y - start.getY());
            if (offset > 0.75) {
                log.append("  previous trajectory rejected as a seed (starts ")
                   .append(fmt(offset)).append(" m from the current pose).\n");
                return Optional.empty();
            }

            double duration = samples.get(samples.size() - 1).t - first.t;
            if (!(duration > 1e-3)) {
                return Optional.empty();
            }

            double startTheta = start.getRotation().getRadians();
            double[][] out = new double[n + 1][6];

            for (int k = 0; k <= n; k++) {
                double t = first.t + duration * k / n;
                SwerveSample s = interpolateSample(samples, t);
                out[k][0] = s.x;
                out[k][1] = s.y;
                // Unwrap against the start heading; SwerveSample stores a wrapped angle and
                // a raw copy would inject a 2*pi discontinuity into the seed.
                out[k][2] = startTheta + MathUtil.angleModulus(s.heading - startTheta);
                out[k][3] = s.vx;
                out[k][4] = s.vy;
                out[k][5] = s.omega;
            }

            log.append("  seeded from the previous trajectory (").append(samples.size())
               .append(" samples, ").append(fmt(duration)).append(" s).\n");
            return Optional.of(out);

        } catch (RuntimeException | LinkageError e) {
            log.append("  previous-trajectory warm start unavailable (").append(e).append(").\n");
            return Optional.empty();
        }
    }

    private SwerveSample interpolateSample(List<SwerveSample> samples, double t) {
        if (t <= samples.get(0).t) {
            return samples.get(0);
        }
        SwerveSample last = samples.get(samples.size() - 1);
        if (t >= last.t) {
            return last;
        }

        int low = 0;
        int high = samples.size() - 1;
        while (high - low > 1) {
            int mid = (low + high) >>> 1;
            if (samples.get(mid).t <= t) {
                low = mid;
            } else {
                high = mid;
            }
        }

        SwerveSample a = samples.get(low);
        SwerveSample b = samples.get(high);
        double span = b.t - a.t;
        if (span < 1e-9) {
            return a;
        }
        return a.interpolate(b, t);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Solution extraction
    // ─────────────────────────────────────────────────────────────────────────────

    private List<TrajectoryPoint> extractSolution(
        Variable totalTime, VariableMatrix x, VariableMatrix u, int n, int numModules
    ) {
        List<TrajectoryPoint> out = new ArrayList<>(n + 1);
        double duration = totalTime.value();
        double dt = duration / n;

        for (int k = 0; k <= n; k++) {
            double theta = x.value(2, k);
            double cosT = Math.cos(theta);
            double sinT = Math.sin(theta);

            double[] fx = new double[numModules];
            double[] fy = new double[numModules];
            double netFx = 0.0;
            double netFy = 0.0;
            double netTau = 0.0;

            // Controls live on intervals. At the terminal knot, report the final interval's
            // force rather than fabricating a new control.
            int uk = Math.min(k, n - 1);

            for (int i = 0; i < numModules; i++) {
                fx[i] = u.value(i, uk);
                fy[i] = u.value(i + numModules, uk);

                Translation2d off = config.offset(i);
                double rx = cosT * off.getX() - sinT * off.getY();
                double ry = sinT * off.getX() + cosT * off.getY();

                netFx += fx[i];
                netFy += fy[i];
                netTau += rx * fy[i] - ry * fx[i];
            }

            out.add(new TrajectoryPoint(
                k * dt,
                x.value(0, k), x.value(1, k), theta,
                x.value(3, k), x.value(4, k), x.value(5, k),
                netFx / config.massKg(), netFy / config.massKg(), netTau / config.moi(),
                fx, fy));
        }

        return out;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Validation (independent of the NLP, against raw physical geometry)
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * @return {@link Optional#empty()} when the trajectory is safe, otherwise a named
     *     {@link Violation}. Every failure identifies the responsible obstacle's type,
     *     index, centre, and penetration depth; the previous boolean return plus a
     *     {@code t=1.000} message made it impossible to tell an interpolated collision from
     *     an illegal knot.
     */
    private Optional<Violation> validateTrajectory(
        List<TrajectoryPoint> trajectory, ObstacleField obstacles, FieldDimensions field,
        double radius, double maxV, double maxOmega, StringBuilder log
    ) {
        if (trajectory == null || trajectory.isEmpty()) {
            return Optional.of(new Violation("EMPTY", "trajectory contains no points"));
        }

        double previousTime = Double.NEGATIVE_INFINITY;

        for (int i = 0; i < trajectory.size(); i++) {
            TrajectoryPoint p = trajectory.get(i);

            if (!isFinite(p)) {
                return Optional.of(new Violation("NON_FINITE", "knot " + i + " has a NaN/Inf field"));
            }
            if (!(p.time() > previousTime) && i > 0) {
                return Optional.of(new Violation("TIME_ORDER",
                    "knot " + i + " time " + fmt(p.time()) + " s is not increasing"));
            }
            previousTime = p.time();

            double speed = Math.hypot(p.vx(), p.vy());
            if (speed > maxV + 0.01) {
                return Optional.of(new Violation("OVERSPEED",
                    "knot " + i + " speed " + fmt(speed) + " m/s exceeds " + fmt(maxV)));
            }
            if (Math.abs(p.omega()) > maxOmega + 0.01) {
                return Optional.of(new Violation("OVER_OMEGA",
                    "knot " + i + " omega " + fmt(p.omega()) + " rad/s exceeds " + fmt(maxOmega)));
            }

            Optional<Violation> atKnot =
                checkPoint(p.x(), p.y(), obstacles, field, radius, "knot " + i);
            if (atKnot.isPresent()) {
                return atKnot;
            }

            if (i == trajectory.size() - 1) {
                break;
            }

            // Interior samples only: s runs 1..samples-1 so a failure can never be reported
            // as t=1.000, which previously hid whether the next knot itself was illegal.
            TrajectoryPoint q = trajectory.get(i + 1);
            double dx = q.x() - p.x();
            double dy = q.y() - p.y();
            int samples = Math.max(1, (int) Math.ceil(Math.hypot(dx, dy) / VALIDATION_SPACING_M));

            for (int s = 1; s < samples; s++) {
                double t = (double) s / samples;
                Optional<Violation> interior = checkPoint(
                    p.x() + t * dx, p.y() + t * dy, obstacles, field, radius,
                    String.format("segment %d->%d at t=%.3f", i, i + 1, t));
                if (interior.isPresent()) {
                    return interior;
                }
            }
        }

        return Optional.empty();
    }

    private Optional<Violation> validateGeometricPath(
        List<Translation2d> path, ObstacleField obstacles, double radius, FieldDimensions field
    ) {
        if (path == null || path.size() < 2) {
            return Optional.of(new Violation("EMPTY", "route has fewer than two points"));
        }

        for (int i = 0; i < path.size(); i++) {
            Translation2d p = path.get(i);
            Optional<Violation> atVertex =
                checkPoint(p.getX(), p.getY(), obstacles, field, radius, "route vertex " + i);
            if (atVertex.isPresent()) {
                return atVertex;
            }
        }

        for (int i = 0; i < path.size() - 1; i++) {
            Translation2d a = path.get(i);
            Translation2d b = path.get(i + 1);
            double dx = b.getX() - a.getX();
            double dy = b.getY() - a.getY();
            int samples = Math.max(1, (int) Math.ceil(Math.hypot(dx, dy) / VALIDATION_SPACING_M));

            for (int s = 1; s < samples; s++) {
                double t = (double) s / samples;
                Optional<Violation> interior = checkPoint(
                    a.getX() + t * dx, a.getY() + t * dy, obstacles, field, radius,
                    String.format("route leg %d->%d at t=%.3f", i, i + 1, t));
                if (interior.isPresent()) {
                    return interior;
                }
            }
        }

        return Optional.empty();
    }

    private Optional<Violation> checkPoint(
        double x, double y, ObstacleField obstacles, FieldDimensions field,
        double radius, String where
    ) {
        if (!isPointWithinField(x, y, field, radius)) {
            return Optional.of(new Violation("FIELD_BOUNDARY",
                where + " at (" + fmt(x) + ", " + fmt(y) + ") is outside the safe field "
                    + "(radius " + fmt(radius) + " m)"));
        }

        Optional<Violation> obstacle = describeObstacleViolation(x, y, obstacles, radius);
        if (obstacle.isPresent()) {
            Violation v = obstacle.get();
            return Optional.of(new Violation(v.kind(), where + " " + v.detail()));
        }

        return Optional.empty();
    }

    /** Names the specific obstacle hit and by how much, in metres. */
    private Optional<Violation> describeObstacleViolation(
        double x, double y, ObstacleField obstacles, double radius
    ) {
        List<CircleObstacle> circles = obstacles.circles();
        for (int i = 0; i < circles.size(); i++) {
            CircleObstacle c = circles.get(i);
            double required = c.radius() + radius;
            double distance = Math.hypot(x - c.x(), y - c.y());
            if (distance + VALIDATION_TOLERANCE_M < required) {
                return Optional.of(new Violation("CIRCLE_COLLISION", String.format(
                    "at (%.3f, %.3f) is %.3f m inside circle #%d centred (%.3f, %.3f) r=%.3f",
                    x, y, required - distance, i, c.x(), c.y(), c.radius())));
            }
        }

        List<RectObstacle> rects = obstacles.rects();
        for (int i = 0; i < rects.size(); i++) {
            RectObstacle r = rects.get(i);
            double dx = Math.max(Math.abs(x - r.x()) - r.width() * 0.5, 0.0);
            double dy = Math.max(Math.abs(y - r.y()) - r.height() * 0.5, 0.0);
            double distance = Math.hypot(dx, dy);
            if (distance + VALIDATION_TOLERANCE_M < radius) {
                return Optional.of(new Violation("RECT_COLLISION", String.format(
                    "at (%.3f, %.3f) is %.3f m inside rect #%d centred (%.3f, %.3f) %.3fx%.3f",
                    x, y, radius - distance, i, r.x(), r.y(), r.width(), r.height())));
            }
        }

        return Optional.empty();
    }

    private boolean isPointWithinField(double x, double y, FieldDimensions field, double radius) {
        return x >= radius - VALIDATION_TOLERANCE_M
            && x <= field.length() - radius + VALIDATION_TOLERANCE_M
            && y >= radius - VALIDATION_TOLERANCE_M
            && y <= field.width() - radius + VALIDATION_TOLERANCE_M;
    }

    private boolean isFinite(TrajectoryPoint p) {
        return Double.isFinite(p.time()) && Double.isFinite(p.x()) && Double.isFinite(p.y())
            && Double.isFinite(p.theta()) && Double.isFinite(p.vx()) && Double.isFinite(p.vy())
            && Double.isFinite(p.omega()) && Double.isFinite(p.ax()) && Double.isFinite(p.ay())
            && Double.isFinite(p.alpha());
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Fallback and trivial trajectories
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * Trapezoidal traversal of an already-validated geometric route.
     *
     * <p>Profiles from the actual current speed, so unlike the previous version it does not
     * command an instantaneous stop when the robot is already moving. Heading rate is
     * populated consistently with the heading ramp, so a follower's rotational feedforward
     * is correct.
     */
    private List<TrajectoryPoint> buildFallbackTrajectory(
        List<Translation2d> route, Pose2d start, double goalTheta,
        ChassisSpeeds startSpeedsField, double maxV, double maxA, int numModules
    ) {
        double totalDist = pathLength(route);
        double startSpeed = Math.hypot(
            startSpeedsField.vxMetersPerSecond, startSpeedsField.vyMetersPerSecond);
        MotionProfile profile = new MotionProfile(totalDist, startSpeed, maxV, maxA);

        double duration = profile.duration();
        int steps = Math.max(16, (int) Math.ceil(totalDist / 0.08));
        double startTheta = start.getRotation().getRadians();
        double headingRate = (goalTheta - startTheta) / duration;

        List<TrajectoryPoint> points = new ArrayList<>(steps + 1);

        for (int k = 0; k <= steps; k++) {
            double t = duration * k / steps;
            double dist = profile.distanceAt(t);
            double speed = profile.speedAt(t);

            Translation2d p = interpolateAtDistance(route, dist);
            Translation2d ahead =
                interpolateAtDistance(route, Math.min(dist + 0.10, totalDist));

            double dx = ahead.getX() - p.getX();
            double dy = ahead.getY() - p.getY();
            double norm = Math.hypot(dx, dy);

            double frac = (double) k / steps;
            boolean endpoint = k == 0 || k == steps;

            points.add(new TrajectoryPoint(
                t,
                p.getX(), p.getY(),
                startTheta + frac * (goalTheta - startTheta),
                norm > 1e-9 ? speed * dx / norm : 0.0,
                norm > 1e-9 ? speed * dy / norm : 0.0,
                endpoint ? 0.0 : headingRate,
                0.0, 0.0, 0.0,
                new double[numModules], new double[numModules]));
        }

        return points;
    }

    /**
     * Two samples rather than one: followers that compute {@code t / totalTime} or index
     * {@code i + 1} divide by zero or throw on a single-sample trajectory.
     */
    private List<TrajectoryPoint> buildStationaryTrajectory(Pose2d pose) {
        int numModules = config.moduleCount();
        double theta = pose.getRotation().getRadians();

        TrajectoryPoint first = new TrajectoryPoint(
            0.0, pose.getX(), pose.getY(), theta,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            new double[numModules], new double[numModules]);

        TrajectoryPoint second = new TrajectoryPoint(
            0.02, pose.getX(), pose.getY(), theta,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            new double[numModules], new double[numModules]);

        return List.of(first, second);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Geometry helpers
    // ─────────────────────────────────────────────────────────────────────────────

    private double pathLength(List<Translation2d> path) {
        double total = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            total += path.get(i).getDistance(path.get(i + 1));
        }
        return total;
    }

    private Translation2d interpolateAtDistance(List<Translation2d> path, double target) {
        if (path == null || path.isEmpty()) {
            return Translation2d.kZero;
        }
        if (path.size() == 1 || target <= 0.0) {
            return path.get(0);
        }

        double travelled = 0.0;
        for (int i = 0; i < path.size() - 1; i++) {
            Translation2d a = path.get(i);
            Translation2d b = path.get(i + 1);
            double length = a.getDistance(b);

            if (travelled + length >= target) {
                return length > 1e-9 ? a.interpolate(b, (target - travelled) / length) : a;
            }
            travelled += length;
        }

        return path.get(path.size() - 1);
    }

    private ChassisSpeeds toFieldRelative(
        ChassisSpeeds speeds, boolean alreadyFieldRelative, Rotation2d heading
    ) {
        if (speeds == null) {
            return new ChassisSpeeds();
        }
        return alreadyFieldRelative
            ? speeds
            : ChassisSpeeds.fromRobotRelativeSpeeds(speeds, heading);
    }

    private ChassisSpeeds sanitizeSpeeds(
        ChassisSpeeds raw, double maxLinear, double maxAngular, StringBuilder log, String label
    ) {
        if (raw == null) {
            return new ChassisSpeeds();
        }

        double vx = finiteOrZero(raw.vxMetersPerSecond, log, label + " vx");
        double vy = finiteOrZero(raw.vyMetersPerSecond, log, label + " vy");
        double omega = finiteOrZero(raw.omegaRadiansPerSecond, log, label + " omega");

        double speed = Math.hypot(vx, vy);
        if (speed > maxLinear && speed > 1e-9) {
            log.append("Clamped ").append(label).append(" from ").append(fmt(speed))
               .append(" to ").append(fmt(maxLinear)).append(" m/s.\n");
            double scale = maxLinear / speed;
            vx *= scale;
            vy *= scale;
        }

        if (maxAngular < Double.MAX_VALUE && Math.abs(omega) > maxAngular) {
            log.append("Clamped ").append(label).append(" omega from ").append(fmt(omega))
               .append(" to ").append(fmt(Math.copySign(maxAngular, omega))).append(" rad/s.\n");
            omega = Math.copySign(maxAngular, omega);
        }

        return new ChassisSpeeds(vx, vy, omega);
    }

    private double finiteOrZero(double value, StringBuilder log, String label) {
        if (Double.isFinite(value)) {
            return value;
        }
        log.append("WARNING: ").append(label).append(" was non-finite; using zero.\n");
        return 0.0;
    }

    private static String fmt(double value) {
        return String.format("%.3f", value);
    }

    private static String fmtPoint(Translation2d p) {
        return String.format("%.3f, %.3f", p.getX(), p.getY());
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Output conversion
    // ─────────────────────────────────────────────────────────────────────────────

    public Pose2d[] reconstructToPose2d(List<TrajectoryPoint> trajectory) {
        if (trajectory == null || trajectory.isEmpty()) {
            return new Pose2d[0];
        }
        Pose2d[] poses = new Pose2d[trajectory.size()];
        for (int i = 0; i < trajectory.size(); i++) {
            poses[i] = trajectory.get(i).pose();
        }
        return poses;
    }

    /** Empty-safe conversion. Prefer this over {@link #convertToChoreo(List)}. */
    public Optional<Trajectory<SwerveSample>> toChoreo(List<TrajectoryPoint> path, String name) {
        if (path == null || path.size() < 2) {
            return Optional.empty();
        }

        List<SwerveSample> samples = new ArrayList<>(path.size());
        for (TrajectoryPoint p : path) {
            samples.add(new SwerveSample(
                p.time(), p.x(), p.y(), p.theta(),
                p.vx(), p.vy(), p.omega(),
                p.ax(), p.ay(), p.alpha(),
                p.moduleForcesX(), p.moduleForcesY()));
        }

        return Optional.of(new Trajectory<>(name, samples, List.of(), List.of()));
    }

    /**
     * @deprecated Returns {@code null} for unusable input, which is easy to dereference by
     *     accident. Use {@link #toChoreo(List, String)}.
     */
    @Deprecated
    public Trajectory<SwerveSample> convertToChoreo(List<TrajectoryPoint> path) {
        return toChoreo(path, "ProPath").orElse(null);
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Logging
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * Appends the diagnostic buffer, rotating the file past {@link #MAX_LOG_BYTES}.
     * Still synchronous file I/O: acceptable only because {@code plan()} is expected to run
     * off the main loop. Do not enable this on a real-time thread.
     */
    private void writeLog(String contents) {
        if (!ENABLE_LOGGING) {
            return;
        }

        try {
            File file = new File(LOG_FILE_PATH);
            boolean truncate = file.exists() && file.length() > MAX_LOG_BYTES;
            try (PrintWriter out = new PrintWriter(new FileWriter(file, !truncate))) {
                if (truncate) {
                    out.println("=== log rotated (exceeded " + MAX_LOG_BYTES + " bytes) ===");
                }
                out.print(contents);
            }
        } catch (IOException e) {
            System.err.println("TrajoPlotPro logging failed: " + e.getMessage());
        }
    }
    public record ConstraintCirc(double x, double y, double rSq, int groupId) {}
}