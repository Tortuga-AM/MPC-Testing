package frc.robot.Drive.AutoAlign;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Drive.AutoAlign.AutoAlignConstants.*;
import static frc.robot.Drive.DriveConstants.ANGULAR_VELOCITY_LIMIT;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Drive.Drive;
import frc.robot.Drive.DriveStates;
import frc.robot.Drive.TunerConstants;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

/**
 * Owns autonomous alignment, pathfinding, trajectory generation, tracking,
 * waypoint behavior, and replanning.
 */
public class AutoAlign extends Subsystem<AutoAlignStates> {
    private static AutoAlign instance;

    private final Drive drive = Drive.getInstance();
    private final Field2d field2d = new Field2d();

    private final ADStar planner = new ADStar(FieldObstacles.createFieldMap());

    private final TrajectoryGenerator trajectoryGenerator =
        new TrajectoryGenerator(
            MAX_LINEAR_SPEED_METERS_PER_SECOND,
            MAX_LINEAR_ACCEL_METERS_PER_SECOND_SQ,
            MAX_LATERAL_ACCEL_METERS_PER_SECOND_SQ,
            MAX_ANGULAR_SPEED_RAD_PER_SECOND,
            MAX_ANGULAR_ACCEL_RAD_PER_SECOND_SQ,
            TRAJECTORY_SAMPLE_SPACING_METERS
        );

    private final HolonomicPoseController poseController =
        new HolonomicPoseController(
            TRANSLATION_KP,
            TRANSLATION_KI,
            TRANSLATION_KD,
            ROTATION_KP,
            ROTATION_KI,
            ROTATION_KD,
            Math.min(
                MAX_LINEAR_SPEED_METERS_PER_SECOND,
                TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
            ),
            Math.min(
                MAX_ANGULAR_SPEED_RAD_PER_SECOND,
                ANGULAR_VELOCITY_LIMIT.in(RadiansPerSecond)
            )
        );

    private final Debouncer atTargetDebouncer =
        new Debouncer(SETTLED_TIME_SECONDS);

    private final List<ConstraintZone> constraintZones =
        new ArrayList<>();

    private final List<ActionableZone> actionableZones =
        new ArrayList<>();

    private List<Waypoint> requestedWaypoints = List.of();
    private int currentWaypointIndex = 0;

    /*
     * targetPose is the requested target.
     * resolvedTargetPose is the nearest safe target returned by AD* when the
     * requested point was inside an obstacle/outside the grid.
     */
    private Pose2d targetPose = null;
    private Pose2d resolvedTargetPose = null;

    private double requestedEndVelocityMetersPerSecond = 0.0;

    private CustomTrajectory activeTrajectory = null;
    private List<Translation2d> activePolyline = List.of();
    private List<CubicBezierSegment> activeBezierSegments = List.of();

    private double trajectoryStartTimeSeconds = 0.0;
    private double lastReplanTimeSeconds = Double.NEGATIVE_INFINITY;

    private Translation2d stuckReferencePosition = Translation2d.kZero;
    private double stuckStartTimeSeconds = 0.0;
    private double lastCommandedSpeedMetersPerSecond = 0.0;

    private double selectedBezierHandleScale = 0.0;
    private String failureReason = "";

    private HeadingRequest headingRequest = HeadingRequest.profiled();

    private boolean releaseDriveAtTrajectoryEnd = false;

    public static AutoAlign getInstance() {
        if (instance == null) {
            instance = new AutoAlign();
        }

        return instance;
    }

    private AutoAlign() {
        super("AutoAlign", AutoAlignStates.IDLE);
    }

    @Override
    protected void stateInit() {
        switch (getState()) {
            case PLANNING, REPLANNING -> {
                drive.setState(DriveStates.AUTO_ALIGN);
                drive.stop();

                activeTrajectory = null;
                activePolyline = List.of();
                activeBezierSegments = List.of();

                poseController.reset();
                atTargetDebouncer.calculate(false);

                stuckReferencePosition = drive.getPose().getTranslation();
                stuckStartTimeSeconds = Timer.getFPGATimestamp();
            }

            case FOLLOWING_TRAJECTORY -> {
                if (activeTrajectory == null) {
                    failureReason = "Tried to follow a null trajectory";
                    setState(AutoAlignStates.FAILED);
                    return;
                }

                poseController.reset();
                atTargetDebouncer.calculate(false);

                trajectoryStartTimeSeconds = Timer.getFPGATimestamp();
                stuckReferencePosition = drive.getPose().getTranslation();
                stuckStartTimeSeconds = trajectoryStartTimeSeconds;

                drive.setState(DriveStates.AUTO_ALIGN);
            }

            case IDLE, FAILED, CANCELLED -> {
                drive.stop();
                drive.setState(DriveStates.MANUAL);
            }

            case AT_TARGET -> {
                /*
                * For a stop trajectory, stop normally.
                *
                * For a nonzero terminal velocity, immediately return command ownership
                * to manual/whatever higher-level command takes over. Do not send a
                * corrective stop command first.
                */
                if (!releaseDriveAtTrajectoryEnd) {
                    drive.stop();
                }

                drive.setState(DriveStates.MANUAL);

                releaseDriveAtTrajectoryEnd = false;
            }
        }
    }

    @Override
    protected void runState() {
        switch (getState()) {
            case IDLE, AT_TARGET, FAILED, CANCELLED -> {
                // No autonomous chassis command.
            }

            case PLANNING, REPLANNING -> runPlanning();

            case FOLLOWING_TRAJECTORY -> runTrajectoryFollower();
        }

        updateTelemetry();
    }

    /** Plan to a pose and stop at it. */
    public void alignToPose(Pose2d targetPose) {
        alignToPose(targetPose, DEFAULT_END_VELOCITY_METERS_PER_SECOND);
    }

    /**
     * Plan to a pose with an optional nonzero terminal speed.
     *
     * <p>Use nonzero terminal speed only if this is a pass-through point or
     * another motion command will immediately take over.
     */
    public void alignToPose(
        Pose2d targetPose,
        double endVelocityMetersPerSecond
    ) {
        boolean shouldStop = endVelocityMetersPerSecond <= 1e-6;

        alignToWaypoints(List.of(
            new Waypoint(
                targetPose,
                shouldStop,
                Math.max(0.0, endVelocityMetersPerSecond),
                () -> {}
            )
        ));
    }

    /**
     * Plan and execute a sequence of required waypoints.
     *
     * <p>Each leg is independently pathfound. This is intentional: it lets
     * each new leg begin from measured pose and measured velocity.
     */
    public void alignToWaypoints(List<Waypoint> waypoints) {
        if (waypoints == null || waypoints.isEmpty()) {
            throw new IllegalArgumentException(
                "AutoAlign requires at least one waypoint"
            );
        }

        requestedWaypoints = List.copyOf(waypoints);
        currentWaypointIndex = 0;
        failureReason = "";

        startPlanningCurrentWaypoint(false);
    }

    /** Compatibility overload if existing code uses alignToPose(List). */
    public void alignToPose(List<Waypoint> waypoints) {
        alignToWaypoints(waypoints);
    }

    public void cancel() {
        setState(AutoAlignStates.CANCELLED);
    }

    public boolean isAligning() {
        return getState() == AutoAlignStates.PLANNING
            || getState() == AutoAlignStates.REPLANNING
            || getState() == AutoAlignStates.FOLLOWING_TRAJECTORY;
    }

    public boolean isAtTarget() {
        return getState() == AutoAlignStates.AT_TARGET;
    }

    public boolean hasFailed() {
        return getState() == AutoAlignStates.FAILED;
    }

    public String getFailureReason() {
        return failureReason;
    }

    /** Returns the requested waypoint pose. */
    public Pose2d getTargetPose() {
        return targetPose;
    }

    /**
     * Returns the actual safe endpoint selected by AD*.
     *
     * <p>This can differ from getTargetPose() if the requested target was
     * inside an obstacle or outside the navgrid.
     */
    public Pose2d getResolvedTargetPose() {
        return resolvedTargetPose;
    }

    public void setConstraintZones(List<ConstraintZone> zones) {
        constraintZones.clear();

        if (zones != null) {
            constraintZones.addAll(zones);
        }
    }

    public void setActionableZones(List<ActionableZone> zones) {
        actionableZones.clear();

        if (zones != null) {
            actionableZones.addAll(zones);
        }
    }

    /**
     * Update dynamic obstacle rectangles.
     *
     * <p>Obstacle rectangles must already be inflated for robot footprint and
     * desired safety margin.
     */
    public void setDynamicObstacles(
        List<ADStar.ObstacleRectangle> obstacles
    ) {
        planner.setDynamicObstacles(
            obstacles,
            drive.getPose().getTranslation()
        );
    }

    private void startPlanningCurrentWaypoint(boolean replanning) {
        if (currentWaypointIndex < 0
            || currentWaypointIndex >= requestedWaypoints.size()) {
            failureReason = "Waypoint index was outside valid range";
            setState(AutoAlignStates.FAILED);
            return;
        }

        Waypoint waypoint = requestedWaypoints.get(currentWaypointIndex);

        targetPose = waypoint.pose();

        requestedEndVelocityMetersPerSecond =
            waypoint.stopAtWaypoint()
                ? 0.0
                : waypoint.passThroughVelocityMetersPerSecond();

        planner.setStartPosition(drive.getPose().getTranslation());
        planner.setGoalPosition(targetPose.getTranslation());

        setState(
            replanning
                ? AutoAlignStates.REPLANNING
                : AutoAlignStates.PLANNING
        );
    }

    private void runPlanning() {
        if (!planner.isNewPathAvailable()) {
            if (getStateTime() > PLANNING_TIMEOUT_SECONDS) {
                failureReason = "Timed out waiting for AD* path";
                setState(AutoAlignStates.FAILED);
            }

            return;
        }

        ADStar.PathResult plannedPath = planner.getLatestPath();

        if (!plannedPath.valid()
            || plannedPath.simplifiedPath().size() < 2) {
            failureReason = "AD* did not find a valid path";
            setState(AutoAlignStates.FAILED);
            return;
        }

        activePolyline = plannedPath.simplifiedPath();

        /*
         * The final simplified point is the actual safe projected endpoint.
         * It may differ from targetPose translation if targetPose was blocked.
         */
        Translation2d resolvedGoalTranslation = activePolyline.get(
            activePolyline.size() - 1
        );

        resolvedTargetPose = new Pose2d(
            resolvedGoalTranslation,
            targetPose.getRotation()
        );

        activeBezierSegments = List.of();
        selectedBezierHandleScale = 0.0;

        /*
         * Try from smoothest to most conservative geometry.
         *
         * scale = 0.0 produces straight anchor-to-anchor cubic segments and
         * remains closest to the AD* simplified polyline.
         */
        for (double handleScale : BEZIER_HANDLE_SCALES) {
            BezierPathGenerator candidateGenerator =
                new BezierPathGenerator(
                    BEZIER_CORNER_ANCHOR_PERCENT,
                    BEZIER_MAX_CORNER_CUT_PERCENT,
                    handleScale
                );

            List<CubicBezierSegment> candidateSegments =
                candidateGenerator.generate(activePolyline);

            if (candidateSegments.isEmpty()) {
                continue;
            }

            activeBezierSegments = candidateSegments;
            selectedBezierHandleScale = handleScale;
        }

        if (activeBezierSegments.isEmpty()) {
            failureReason =
                "No collision-free Bezier candidate found after retries";

            setState(AutoAlignStates.FAILED);
            return;
        }

        try {
            activeTrajectory = trajectoryGenerator.generate(
                activeBezierSegments,
                drive.getPose().getRotation(),
                resolvedTargetPose.getRotation(),
                getFieldRelativeVelocity(),
                requestedEndVelocityMetersPerSecond,
                constraintZones,
                headingRequest
            );

            releaseDriveAtTrajectoryEnd =
                requestedEndVelocityMetersPerSecond > 1e-4;
        } catch (Exception exception) {
            failureReason =
                "Trajectory generation failed: "
                    + exception.getMessage();

            setState(AutoAlignStates.FAILED);
            return;
        }

        logPlannedPath();
        setState(AutoAlignStates.FOLLOWING_TRAJECTORY);
    }

    private void runTrajectoryFollower() {
        if (activeTrajectory == null || resolvedTargetPose == null) {
            failureReason = "Trajectory or resolved target was missing";
            setState(AutoAlignStates.FAILED);
            return;
        }

        double now = Timer.getFPGATimestamp();
        double trajectoryTimeSeconds = now - trajectoryStartTimeSeconds;

        TrajectorySample desiredState =
            activeTrajectory.sample(trajectoryTimeSeconds);

        ChassisSpeeds command = poseController.calculate(
            drive.getPose(),
            desiredState.pose(),
            desiredState.fieldVelocityMetersPerSecond(),
            desiredState.angularVelocityRadPerSecond()
        );

        drive.driveRobotRelative(command);

        /*
        * A nonzero terminal velocity is a handoff condition.
        *
        * Do not wait for pose PID to settle at the final position, because that
        * would make the robot correct backward after it has intentionally passed
        * through the final point.
        */
        if (requestedEndVelocityMetersPerSecond > 1e-4
            && trajectoryTimeSeconds >= activeTrajectory.getTotalTimeSeconds()) {
            setState(AutoAlignStates.AT_TARGET);
            return;
        }

        lastCommandedSpeedMetersPerSecond = Math.hypot(
            command.vxMetersPerSecond,
            command.vyMetersPerSecond
        );

        /*
         * Compare only against the current/future region of the trajectory.
         * Otherwise the robot could be near an old part of a looping path and
         * incorrectly appear to be tracking correctly.
         */
        double crossTrackError = getClosestTrajectoryDistance(
            drive.getPose().getTranslation(),
            trajectoryTimeSeconds
        );

        Logger.recordOutput(
            "AutoAlign/CrossTrackErrorMeters",
            crossTrackError
        );

        if (trajectoryTimeSeconds > REPLAN_GRACE_PERIOD_SECONDS
            && crossTrackError > REPLAN_CROSS_TRACK_ERROR_METERS) {
            requestReplan("Cross-track error exceeded threshold");
            return;
        }

        if (isStuck(now)) {
            requestReplan("Robot appears stuck");
            return;
        }

        boolean trajectoryFinished =
            trajectoryTimeSeconds >= activeTrajectory.getTotalTimeSeconds();

        boolean atWaypoint = poseController.atTarget(
            drive.getPose(),
            resolvedTargetPose,
            WAYPOINT_POSITION_TOLERANCE_METERS,
            WAYPOINT_ROTATION_TOLERANCE_RADIANS
        );

        if (trajectoryFinished && atWaypoint) {
            onWaypointReached();
            return;
        }

        if (trajectoryTimeSeconds
            > activeTrajectory.getTotalTimeSeconds()
                + MAX_TRAJECTORY_OVERRUN_SECONDS) {
            requestReplan("Trajectory timed out before waypoint");
        }
    }

    private void onWaypointReached() {
        Waypoint waypoint = requestedWaypoints.get(currentWaypointIndex);

        /*
         * Callback runs on the normal robot periodic thread.
         * Do not put blocking work in this callback.
         */
        waypoint.onReached().run();

        boolean isFinalWaypoint =
            currentWaypointIndex == requestedWaypoints.size() - 1;

        if (!isFinalWaypoint) {
            currentWaypointIndex++;
            startPlanningCurrentWaypoint(true);
            return;
        }

        /*
         * Stop waypoints require the tight final tolerances and settle
         * debounce. A pass-through final waypoint completes immediately once
         * it reaches waypoint tolerance.
         */
        if (!waypoint.stopAtWaypoint()) {
            setState(AutoAlignStates.AT_TARGET);
            return;
        }

        boolean atFinalTarget = poseController.atTarget(
            drive.getPose(),
            resolvedTargetPose,
            POSITION_TOLERANCE_METERS,
            ROTATION_TOLERANCE_RADIANS
        );

        if (atTargetDebouncer.calculate(atFinalTarget)) {
            setState(AutoAlignStates.AT_TARGET);
        }
    }

    private void requestReplan(String reason) {
        double now = Timer.getFPGATimestamp();

        if (now - lastReplanTimeSeconds
            < MIN_REPLAN_INTERVAL_SECONDS) {
            return;
        }

        lastReplanTimeSeconds = now;

        Logger.recordOutput("AutoAlign/ReplanReason", reason);

        /*
         * Replans from actual measured pose and actual measured velocity.
         */
        startPlanningCurrentWaypoint(true);
    }

    private boolean isStuck(double nowSeconds) {
        ChassisSpeeds measuredSpeeds = drive.getRobotRelativeSpeeds();

        double measuredSpeedMetersPerSecond = Math.hypot(
            measuredSpeeds.vxMetersPerSecond,
            measuredSpeeds.vyMetersPerSecond
        );

        Translation2d currentPosition = drive.getPose().getTranslation();

        /*
         * Reset the stuck timer if we are not commanding meaningful movement,
         * or the robot is actually moving.
         */
        if (lastCommandedSpeedMetersPerSecond
                < STUCK_COMMAND_SPEED_THRESHOLD_METERS_PER_SECOND
            || measuredSpeedMetersPerSecond
                > STUCK_MEASURED_SPEED_THRESHOLD_METERS_PER_SECOND) {
            stuckReferencePosition = currentPosition;
            stuckStartTimeSeconds = nowSeconds;
            return false;
        }

        double progressMeters = currentPosition.getDistance(
            stuckReferencePosition
        );

        if (progressMeters >= STUCK_MIN_PROGRESS_METERS) {
            stuckReferencePosition = currentPosition;
            stuckStartTimeSeconds = nowSeconds;
            return false;
        }

        return nowSeconds - stuckStartTimeSeconds
            >= STUCK_DETECTION_TIME_SECONDS;
    }

    private Translation2d getFieldRelativeVelocity() {
        ChassisSpeeds robotRelative = drive.getRobotRelativeSpeeds();

        return new Translation2d(
            robotRelative.vxMetersPerSecond,
            robotRelative.vyMetersPerSecond
        ).rotateBy(drive.getPose().getRotation());
    }

    private double getClosestTrajectoryDistance(
        Translation2d robotPosition,
        double currentTrajectoryTimeSeconds
    ) {
        if (activeTrajectory == null) {
            return Double.POSITIVE_INFINITY;
        }

        double closestDistance = Double.POSITIVE_INFINITY;

        for (TrajectorySample sample : activeTrajectory.getSamples()) {
            /*
             * Include a small amount of history for normal measurement lag,
             * but ignore the distant already-completed path.
             */
            if (sample.timeSeconds()
                < currentTrajectoryTimeSeconds - 0.25) {
                continue;
            }

            closestDistance = Math.min(
                closestDistance,
                robotPosition.getDistance(sample.pose().getTranslation())
            );
        }

        return closestDistance;
    }

    private void logPlannedPath() {
        Pose2d[] polylinePoses = activePolyline.stream()
            .map(point -> new Pose2d(point, Rotation2d.kZero))
            .toArray(Pose2d[]::new);

        field2d.getObject("ADStar Polyline").setPoses(polylinePoses);

        if (activeTrajectory != null) {
            Pose2d[] trajectoryPoses = activeTrajectory.getSamples()
                .stream()
                .map(TrajectorySample::pose)
                .toArray(Pose2d[]::new);

            field2d.getObject("Bezier Trajectory").setPoses(
                trajectoryPoses
            );
        }
    }

    public void setHeadingRequest(HeadingRequest headingRequest) {
        this.headingRequest = headingRequest == null
            ? HeadingRequest.profiled()
            : headingRequest;
    }

    public HeadingRequest getHeadingRequest() {
        return headingRequest;
    }

    private void updateTelemetry() {
        Pose2d robotPose = drive.getPose();
        field2d.setRobotPose(robotPose);

        for (ActionableZone zone : actionableZones) {
            zone.update(robotPose.getTranslation());

            Logger.recordOutput(
                "AutoAlign/ActionableZones/" + zone.getName() + "/Inside",
                zone.isCurrentlyInside()
            );
        }

        if (targetPose != null) {
            field2d.getObject("Requested Target").setPose(targetPose);
            Logger.recordOutput("AutoAlign/RequestedTarget", targetPose);
        }

        if (resolvedTargetPose != null) {
            field2d.getObject("Resolved Target").setPose(resolvedTargetPose);
            Logger.recordOutput(
                "AutoAlign/ResolvedTarget",
                resolvedTargetPose
            );
        }

        planner.logToAdvantageKit("AutoAlign/Planner");

        Logger.recordOutput(
            "AutoAlign/State",
            getState().getStateString()
        );

        Logger.recordOutput(
            "AutoAlign/FailureReason",
            failureReason
        );

        Logger.recordOutput(
            "AutoAlign/WaypointIndex",
            currentWaypointIndex
        );

        Logger.recordOutput(
            "AutoAlign/WaypointCount",
            requestedWaypoints.size()
        );

        Logger.recordOutput(
            "AutoAlign/RequestedEndVelocityMetersPerSecond",
            requestedEndVelocityMetersPerSecond
        );

        Logger.recordOutput(
            "AutoAlign/SelectedBezierHandleScale",
            selectedBezierHandleScale
        );

        Logger.recordOutput(
            "AutoAlign/ActivePolyline",
            activePolyline.toArray(new Translation2d[0])
        );

        Logger.recordOutput(
            "AutoAlign/BezierSegmentCount",
            activeBezierSegments.size()
        );

        Logger.recordOutput(
            "AutoAlign/TrajectoryTotalTimeSeconds",
            activeTrajectory == null
                ? 0.0
                : activeTrajectory.getTotalTimeSeconds()
        );

        Logger.recordOutput(
            "AutoAlign/TrajectoryElapsedTimeSeconds",
            getState() == AutoAlignStates.FOLLOWING_TRAJECTORY
                ? Timer.getFPGATimestamp() - trajectoryStartTimeSeconds
                : 0.0
        );

        SmartDashboard.putData("AutoAlign/Field", field2d);

        SmartDashboard.putString(
            "AutoAlign/State",
            getState().getStateString()
        );

        SmartDashboard.putString(
            "AutoAlign/FailureReason",
            failureReason
        );

        SmartDashboard.putNumber(
            "AutoAlign/PlannerTimeSeconds",
            planner.getLastPlanningTimeSeconds()
        );

        SmartDashboard.putNumber(
            "AutoAlign/BezierHandleScale",
            selectedBezierHandleScale
        );
    }
}