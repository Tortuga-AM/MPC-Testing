package frc.robot.Drive.AutoAlign;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

/**
 * A sampled, time-parameterized holonomic trajectory.
 */
public class CustomTrajectory {
    private final List<TrajectorySample> samples;

    public CustomTrajectory(List<TrajectorySample> samples) {
        if (samples == null || samples.isEmpty()) {
            throw new IllegalArgumentException("Trajectory requires at least one sample");
        }

        this.samples = new ArrayList<>(samples);
    }

    public double getTotalTimeSeconds() {
        return samples.get(samples.size() - 1).timeSeconds();
    }

    public Pose2d getInitialPose() {
        return samples.get(0).pose();
    }

    public Pose2d getFinalPose() {
        return samples.get(samples.size() - 1).pose();
    }

    public List<TrajectorySample> getSamples() {
        return List.copyOf(samples);
    }

    /**
     * Sample and linearly interpolate the trajectory at a time.
     *
     * @param timeSeconds Trajectory time
     * @return Interpolated desired trajectory state
     */
    public TrajectorySample sample(double timeSeconds) {
        if (timeSeconds <= 0.0) {
            return samples.get(0);
        }

        if (timeSeconds >= getTotalTimeSeconds()) {
            return samples.get(samples.size() - 1);
        }

        for (int i = 0; i < samples.size() - 1; i++) {
            TrajectorySample before = samples.get(i);
            TrajectorySample after = samples.get(i + 1);

            if (timeSeconds >= before.timeSeconds()
                    && timeSeconds <= after.timeSeconds()) {

                double segmentTime = after.timeSeconds() - before.timeSeconds();

                double t = segmentTime <= 1e-9
                    ? 0.0
                    : (timeSeconds - before.timeSeconds()) / segmentTime;

                Pose2d interpolatedPose = before.pose().interpolate(after.pose(), t);

                Translation2d interpolatedVelocity =
                    before.fieldVelocityMetersPerSecond().interpolate(
                        after.fieldVelocityMetersPerSecond(),
                        t
                    );

                double interpolatedOmega = MathUtil.interpolate(
                    before.angularVelocityRadPerSecond(),
                    after.angularVelocityRadPerSecond(),
                    t
                );

                return new TrajectorySample(
                    timeSeconds,
                    interpolatedPose,
                    interpolatedVelocity,
                    interpolatedOmega
                );
            }
        }

        return samples.get(samples.size() - 1);
    }
}