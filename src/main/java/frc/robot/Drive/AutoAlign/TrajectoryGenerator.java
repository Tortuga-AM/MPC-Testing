package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.ArrayList;
import java.util.List;

/**
 * Creates time-parameterized trajectories from cubic Bezier segments.
 *
 * <p>Translation follows the Bezier path tangent. Robot heading behavior is
 * selected with HeadingRequest.
 */
public class TrajectoryGenerator {
    private static final double DEFAULT_SAMPLE_SPACING_METERS = 0.04;
    private static final double MIN_SAMPLE_SPACING_METERS = 0.01;
    private static final double MIN_VELOCITY_FOR_TIME_CALCULATION = 1e-5;
    private static final double EPSILON = 1e-9;
    private static final double HOLD_SAMPLE_PERIOD_SECONDS = 0.02;

    private final double maxLinearVelocityMetersPerSecond;
    private final double maxLinearAccelerationMetersPerSecondSq;
    private final double maxLateralAccelerationMetersPerSecondSq;
    private final double maxAngularVelocityRadPerSecond;
    private final double maxAngularAccelerationRadPerSecondSq;
    private final double sampleSpacingMeters;

    public TrajectoryGenerator(
        double maxLinearVelocityMetersPerSecond,
        double maxLinearAccelerationMetersPerSecondSq,
        double maxLateralAccelerationMetersPerSecondSq,
        double maxAngularVelocityRadPerSecond,
        double maxAngularAccelerationRadPerSecondSq
    ) {
        this(
            maxLinearVelocityMetersPerSecond,
            maxLinearAccelerationMetersPerSecondSq,
            maxLateralAccelerationMetersPerSecondSq,
            maxAngularVelocityRadPerSecond,
            maxAngularAccelerationRadPerSecondSq,
            DEFAULT_SAMPLE_SPACING_METERS
        );
    }

    public TrajectoryGenerator(
        double maxLinearVelocityMetersPerSecond,
        double maxLinearAccelerationMetersPerSecondSq,
        double maxLateralAccelerationMetersPerSecondSq,
        double maxAngularVelocityRadPerSecond,
        double maxAngularAccelerationRadPerSecondSq,
        double sampleSpacingMeters
    ) {
        if (maxLinearVelocityMetersPerSecond <= 0.0
            || maxLinearAccelerationMetersPerSecondSq <= 0.0
            || maxLateralAccelerationMetersPerSecondSq <= 0.0
            || maxAngularVelocityRadPerSecond <= 0.0
            || maxAngularAccelerationRadPerSecondSq <= 0.0) {
            throw new IllegalArgumentException(
                "Trajectory constraints must all be positive"
            );
        }

        this.maxLinearVelocityMetersPerSecond =
            maxLinearVelocityMetersPerSecond;

        this.maxLinearAccelerationMetersPerSecondSq =
            maxLinearAccelerationMetersPerSecondSq;

        this.maxLateralAccelerationMetersPerSecondSq =
            maxLateralAccelerationMetersPerSecondSq;

        this.maxAngularVelocityRadPerSecond =
            maxAngularVelocityRadPerSecond;

        this.maxAngularAccelerationRadPerSecondSq =
            maxAngularAccelerationRadPerSecondSq;

        this.sampleSpacingMeters = Math.max(
            sampleSpacingMeters,
            MIN_SAMPLE_SPACING_METERS
        );
    }

    /**
     * Basic rest-to-rest trajectory using the original profiled heading mode.
     */
    public CustomTrajectory generate(
        List<CubicBezierSegment> segments,
        Rotation2d startRotation,
        Rotation2d endRotation
    ) {
        return generate(
            segments,
            startRotation,
            endRotation,
            Translation2d.kZero,
            0.0,
            List.of(),
            HeadingRequest.profiled()
        );
    }

    /**
     * Generate a trajectory with boundary velocity, constraint-zone, and
     * heading-control support.
     *
     * @param segments Connected Bezier path segments
     * @param startRotation Current robot heading
     * @param endRotation Desired heading at path end
     * @param initialFieldVelocity Measured robot field-relative velocity
     * @param endPathVelocityMetersPerSecond Desired terminal velocity along the
     *     final path tangent
     * @param constraintZones Optional velocity/acceleration constraint zones
     * @param headingRequest Desired heading-control mode
     */
    public CustomTrajectory generate(
        List<CubicBezierSegment> segments,
        Rotation2d startRotation,
        Rotation2d endRotation,
        Translation2d initialFieldVelocity,
        double endPathVelocityMetersPerSecond,
        List<ConstraintZone> constraintZones,
        HeadingRequest headingRequest
    ) {
        if (segments == null || segments.isEmpty()) {
            throw new IllegalArgumentException(
                "Cannot generate a trajectory with no Bezier segments"
            );
        }

        if (headingRequest == null) {
            headingRequest = HeadingRequest.profiled();
        }

        List<GeometrySample> samples = sampleBezierGeometry(segments);

        if (samples.size() < 2) {
            throw new IllegalArgumentException(
                "Bezier path did not produce enough samples"
            );
        }

        for (GeometrySample sample : samples) {
            sample.maxAccelerationMetersPerSecondSq =
                maxLinearAccelerationMetersPerSecondSq;
        }

        double initialPathVelocity = getInitialPathVelocity(
            initialFieldVelocity,
            samples.get(0).tangent
        );

        double boundedEndVelocity = MathUtil.clamp(
            endPathVelocityMetersPerSecond,
            0.0,
            maxLinearVelocityMetersPerSecond
        );

        applyVelocityLimits(
            samples,
            initialPathVelocity,
            boundedEndVelocity,
            constraintZones
        );

        runForwardAccelerationPass(samples);
        runBackwardDecelerationPass(samples);
        assignTimestamps(samples);

        double translationDuration =
            samples.get(samples.size() - 1).timeSeconds;

        double totalDistanceMeters =
            samples.get(samples.size() - 1).distanceMeters;

        RotationProfile rotationProfile = createRotationProfile(
            startRotation,
            endRotation
        );

        boolean endingWithVelocity = boundedEndVelocity > EPSILON;

        /*
         * A nonzero terminal velocity means the path cannot pause at the end
         * just to finish a slow angular profile.
         */
        if (endingWithVelocity
            && headingRequest.mode() == HeadingMode.PROFILED
            && rotationProfile.durationSeconds > translationDuration + EPSILON) {
            throw new IllegalArgumentException(
                "Profiled heading cannot reach final rotation before a "
                    + "nonzero-velocity trajectory ends. Use INTERPOLATE, "
                    + "PATH_TANGENT, FUNCTION, reduce end velocity, or reduce "
                    + "the required heading change."
            );
        }

        /*
         * Only PROFILED + stop trajectories may hold position at the end while
         * finishing their heading motion.
         */
        double totalDuration =
            !endingWithVelocity
                && headingRequest.mode() == HeadingMode.PROFILED
                    ? Math.max(
                        translationDuration,
                        rotationProfile.durationSeconds
                    )
                    : translationDuration;

        List<TrajectorySample> trajectorySamples = new ArrayList<>();

        for (GeometrySample sample : samples) {
            HeadingState heading = calculateHeading(
                headingRequest,
                sample,
                startRotation,
                endRotation,
                totalDistanceMeters,
                translationDuration,
                rotationProfile
            );

            trajectorySamples.add(new TrajectorySample(
                sample.timeSeconds,
                new Pose2d(sample.position, heading.rotation),
                new Translation2d(
                    sample.velocityMetersPerSecond,
                    sample.tangent
                ),
                heading.angularVelocityRadPerSecond
            ));
        }

        GeometrySample finalSample = samples.get(samples.size() - 1);

        /*
         * For a stop trajectory using PROFILED heading, hold position and
         * complete the independent angular profile.
         */
        if (!endingWithVelocity
            && headingRequest.mode() == HeadingMode.PROFILED) {
            for (
                double time = translationDuration + HOLD_SAMPLE_PERIOD_SECONDS;
                time < totalDuration - EPSILON;
                time += HOLD_SAMPLE_PERIOD_SECONDS
            ) {
                HeadingState heading = calculateHeading(
                    headingRequest,
                    finalSample,
                    startRotation,
                    endRotation,
                    totalDistanceMeters,
                    translationDuration,
                    rotationProfile,
                    time
                );

                trajectorySamples.add(new TrajectorySample(
                    time,
                    new Pose2d(finalSample.position, heading.rotation),
                    Translation2d.kZero,
                    heading.angularVelocityRadPerSecond
                ));
            }
        }

        /*
         * This final state is the nonzero-end-velocity fix.
         *
         * The old generator always inserted 0.0 here, which forced the follower
         * to brake regardless of the requested terminal velocity.
         */
        HeadingState finalHeading = calculateHeading(
            headingRequest,
            finalSample,
            startRotation,
            endRotation,
            totalDistanceMeters,
            translationDuration,
            rotationProfile,
            totalDuration
        );

        trajectorySamples.add(new TrajectorySample(
            totalDuration,
            new Pose2d(finalSample.position, finalHeading.rotation),
            endingWithVelocity
                ? new Translation2d(
                    finalSample.velocityMetersPerSecond,
                    finalSample.tangent
                )
                : Translation2d.kZero,
            finalHeading.angularVelocityRadPerSecond
        ));

        validateAngularLimits(trajectorySamples);

        return new CustomTrajectory(trajectorySamples);
    }

    private List<GeometrySample> sampleBezierGeometry(
        List<CubicBezierSegment> segments
    ) {
        List<GeometrySample> samples = new ArrayList<>();

        Translation2d previousPosition = null;
        double cumulativeDistanceMeters = 0.0;

        for (int segmentIndex = 0;
             segmentIndex < segments.size();
             segmentIndex++) {

            CubicBezierSegment segment = segments.get(segmentIndex);

            double approximateLengthMeters = segment.approximateLength(100);

            int subdivisions = Math.max(
                2,
                (int) Math.ceil(
                    approximateLengthMeters / sampleSpacingMeters
                )
            );

            for (int i = 0; i <= subdivisions; i++) {
                if (segmentIndex > 0 && i == 0) {
                    continue;
                }

                double t = (double) i / subdivisions;
                PathSample pathSample = segment.sample(t);

                if (previousPosition != null) {
                    cumulativeDistanceMeters += previousPosition.getDistance(
                        pathSample.position()
                    );
                }

                samples.add(new GeometrySample(
                    pathSample.position(),
                    pathSample.tangent(),
                    pathSample.curvatureRadPerMeter(),
                    cumulativeDistanceMeters
                ));

                previousPosition = pathSample.position();
            }
        }

        return samples;
    }

    private double getInitialPathVelocity(
        Translation2d initialFieldVelocity,
        Rotation2d initialPathTangent
    ) {
        double projectedVelocity =
            initialFieldVelocity.getX() * initialPathTangent.getCos()
                + initialFieldVelocity.getY()
                    * initialPathTangent.getSin();

        if (projectedVelocity
            < AutoAlignConstants.INITIAL_SPEED_DIRECTION_DOT_THRESHOLD) {
            return 0.0;
        }

        return Math.min(
            projectedVelocity,
            maxLinearVelocityMetersPerSecond
        );
    }

    private void applyVelocityLimits(
        List<GeometrySample> samples,
        double initialPathVelocity,
        double endPathVelocityMetersPerSecond,
        List<ConstraintZone> constraintZones
    ) {
        for (GeometrySample sample : samples) {
            double localMaxVelocity = maxLinearVelocityMetersPerSecond;

            double localMaxLateralAcceleration =
                maxLateralAccelerationMetersPerSecondSq;

            if (constraintZones != null) {
                for (ConstraintZone zone : constraintZones) {
                    if (zone.contains(sample.position)) {
                        localMaxVelocity = Math.min(
                            localMaxVelocity,
                            zone.getMaxVelocityMetersPerSecond()
                        );

                        localMaxLateralAcceleration = Math.min(
                            localMaxLateralAcceleration,
                            zone.getMaxLateralAccelerationMetersPerSecondSq()
                        );

                        sample.maxAccelerationMetersPerSecondSq = Math.min(
                            sample.maxAccelerationMetersPerSecondSq,
                            zone.getMaxAccelerationMetersPerSecondSq()
                        );
                    }
                }
            }

            double curvatureMagnitude = Math.abs(
                sample.curvatureRadPerMeter
            );

            double curvatureVelocityLimit = localMaxVelocity;

            if (curvatureMagnitude > EPSILON) {
                curvatureVelocityLimit = Math.sqrt(
                    localMaxLateralAcceleration / curvatureMagnitude
                );
            }

            sample.maxVelocityMetersPerSecond = Math.min(
                localMaxVelocity,
                curvatureVelocityLimit
            );

            sample.velocityMetersPerSecond =
                sample.maxVelocityMetersPerSecond;
        }

        GeometrySample first = samples.get(0);
        GeometrySample last = samples.get(samples.size() - 1);

        first.velocityMetersPerSecond = Math.min(
            Math.max(0.0, initialPathVelocity),
            first.maxVelocityMetersPerSecond
        );

        last.velocityMetersPerSecond = Math.min(
            Math.max(0.0, endPathVelocityMetersPerSecond),
            last.maxVelocityMetersPerSecond
        );
    }

    private void runForwardAccelerationPass(List<GeometrySample> samples) {
        for (int i = 1; i < samples.size(); i++) {
            GeometrySample previous = samples.get(i - 1);
            GeometrySample current = samples.get(i);

            double deltaDistance =
                current.distanceMeters - previous.distanceMeters;

            double acceleration = Math.min(
                previous.maxAccelerationMetersPerSecondSq,
                current.maxAccelerationMetersPerSecondSq
            );

            double reachableVelocity = Math.sqrt(Math.max(
                0.0,
                previous.velocityMetersPerSecond
                    * previous.velocityMetersPerSecond
                    + 2.0 * acceleration * deltaDistance
            ));

            current.velocityMetersPerSecond = Math.min(
                current.velocityMetersPerSecond,
                reachableVelocity
            );
        }
    }

    private void runBackwardDecelerationPass(List<GeometrySample> samples) {
        for (int i = samples.size() - 2; i >= 0; i--) {
            GeometrySample current = samples.get(i);
            GeometrySample next = samples.get(i + 1);

            double deltaDistance =
                next.distanceMeters - current.distanceMeters;

            double acceleration = Math.min(
                current.maxAccelerationMetersPerSecondSq,
                next.maxAccelerationMetersPerSecondSq
            );

            double reachableVelocity = Math.sqrt(Math.max(
                0.0,
                next.velocityMetersPerSecond
                    * next.velocityMetersPerSecond
                    + 2.0 * acceleration * deltaDistance
            ));

            current.velocityMetersPerSecond = Math.min(
                current.velocityMetersPerSecond,
                reachableVelocity
            );
        }
    }

    private void assignTimestamps(List<GeometrySample> samples) {
        samples.get(0).timeSeconds = 0.0;

        for (int i = 1; i < samples.size(); i++) {
            GeometrySample previous = samples.get(i - 1);
            GeometrySample current = samples.get(i);

            double distance =
                current.distanceMeters - previous.distanceMeters;

            double velocitySum =
                previous.velocityMetersPerSecond
                    + current.velocityMetersPerSecond;

            double deltaTime = velocitySum
                < MIN_VELOCITY_FOR_TIME_CALCULATION
                    ? 0.0
                    : (2.0 * distance) / velocitySum;

            current.timeSeconds = previous.timeSeconds + deltaTime;
        }
    }

    private RotationProfile createRotationProfile(
        Rotation2d startRotation,
        Rotation2d endRotation
    ) {
        double headingDelta = MathUtil.angleModulus(
            endRotation.getRadians() - startRotation.getRadians()
        );

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxAngularVelocityRadPerSecond,
                maxAngularAccelerationRadPerSecondSq
            )
        );

        TrapezoidProfile.State initial =
            new TrapezoidProfile.State(0.0, 0.0);

        TrapezoidProfile.State goal =
            new TrapezoidProfile.State(headingDelta, 0.0);

        double duration = calculateRestToRestProfileTime(
            Math.abs(headingDelta),
            maxAngularVelocityRadPerSecond,
            maxAngularAccelerationRadPerSecondSq
        );

        return new RotationProfile(profile, initial, goal, duration);
    }

    private HeadingState calculateHeading(
        HeadingRequest request,
        GeometrySample sample,
        Rotation2d startRotation,
        Rotation2d endRotation,
        double totalDistanceMeters,
        double translationDurationSeconds,
        RotationProfile rotationProfile
    ) {
        return calculateHeading(
            request,
            sample,
            startRotation,
            endRotation,
            totalDistanceMeters,
            translationDurationSeconds,
            rotationProfile,
            sample.timeSeconds
        );
    }

    private HeadingState calculateHeading(
        HeadingRequest request,
        GeometrySample sample,
        Rotation2d startRotation,
        Rotation2d endRotation,
        double totalDistanceMeters,
        double translationDurationSeconds,
        RotationProfile rotationProfile,
        double timeSeconds
    ) {
        return switch (request.mode()) {
            case PROFILED -> {
                TrapezoidProfile.State state =
                    rotationProfile.profile.calculate(
                        Math.min(timeSeconds, rotationProfile.durationSeconds),
                        rotationProfile.initial,
                        rotationProfile.goal
                    );

                yield new HeadingState(
                    Rotation2d.fromRadians(
                        startRotation.getRadians() + state.position
                    ),
                    state.velocity
                );
            }

            case INTERPOLATE -> {
                double progress = totalDistanceMeters < EPSILON
                    ? 1.0
                    : sample.distanceMeters / totalDistanceMeters;

                Rotation2d heading = startRotation.interpolate(
                    endRotation,
                    progress
                );

                double headingDelta = MathUtil.angleModulus(
                    endRotation.getRadians() - startRotation.getRadians()
                );

                double omega = totalDistanceMeters < EPSILON
                    ? 0.0
                    : headingDelta
                        * sample.velocityMetersPerSecond
                        / totalDistanceMeters;

                yield new HeadingState(heading, omega);
            }

            case PATH_TANGENT -> new HeadingState(
                sample.tangent,
                sample.curvatureRadPerMeter
                    * sample.velocityMetersPerSecond
            );

            case FUNCTION -> {
                double progress = totalDistanceMeters < EPSILON
                    ? 1.0
                    : sample.distanceMeters / totalDistanceMeters;

                HeadingContext context = new HeadingContext(
                    sample.position,
                    sample.tangent,
                    sample.distanceMeters,
                    totalDistanceMeters,
                    progress,
                    timeSeconds,
                    translationDurationSeconds
                );

                Rotation2d heading = request.function().getHeading(context);

                /*
                 * Angular velocity is estimated after all samples are created.
                 * Returning zero here is fine for now.
                 */
                yield new HeadingState(heading, 0.0);
            }
        };
    }

    /**
     * Check heading-rate limits. FUNCTION heading mode receives an estimated
     * omega afterward because the function may be arbitrary.
     */
    private void validateAngularLimits(List<TrajectorySample> samples) {
        if (samples.size() < 2) {
            return;
        }

        for (int i = 1; i < samples.size(); i++) {
            TrajectorySample previous = samples.get(i - 1);
            TrajectorySample current = samples.get(i);

            double deltaTime =
                current.timeSeconds() - previous.timeSeconds();

            if (deltaTime <= EPSILON) {
                continue;
            }

            double headingDelta = MathUtil.angleModulus(
                current.pose().getRotation().getRadians()
                    - previous.pose().getRotation().getRadians()
            );

            double estimatedOmega = headingDelta / deltaTime;

            if (Math.abs(estimatedOmega)
                > maxAngularVelocityRadPerSecond + 1e-6) {
                throw new IllegalArgumentException(
                    "Heading mode exceeds max angular velocity. "
                        + "Slow translation, reduce heading change, or "
                        + "increase angular constraints."
                );
            }
        }
    }

    private double calculateRestToRestProfileTime(
        double distance,
        double maxVelocity,
        double maxAcceleration
    ) {
        if (distance <= EPSILON) {
            return 0.0;
        }

        double fullTrapezoidDistance =
            (maxVelocity * maxVelocity) / maxAcceleration;

        if (distance >= fullTrapezoidDistance) {
            double accelerationTime = maxVelocity / maxAcceleration;
            double cruiseDistance =
                distance - fullTrapezoidDistance;

            return 2.0 * accelerationTime
                + cruiseDistance / maxVelocity;
        }

        return 2.0 * Math.sqrt(distance / maxAcceleration);
    }

    private static class GeometrySample {
        final Translation2d position;
        final Rotation2d tangent;
        final double curvatureRadPerMeter;
        final double distanceMeters;

        double maxVelocityMetersPerSecond;
        double velocityMetersPerSecond;
        double maxAccelerationMetersPerSecondSq;
        double timeSeconds;

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

    private record RotationProfile(
        TrapezoidProfile profile,
        TrapezoidProfile.State initial,
        TrapezoidProfile.State goal,
        double durationSeconds
    ) {}

    private record HeadingState(
        Rotation2d rotation,
        double angularVelocityRadPerSecond
    ) {}
}