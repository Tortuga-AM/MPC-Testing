package frc.robot.Drive.AutoAlign;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SpatialTrajectorySampler {
    private final Trajectory<SwerveSample> trajectory;
    private double lastFoundTime = 0.0;

    public SpatialTrajectorySampler(Trajectory<SwerveSample> trajectory, Pose2d initialRobotPose) {
        this.trajectory = trajectory;
        
        // --- GLOBAL INITIALIZATION SEARCH ---
        double minDistance = Double.MAX_VALUE;
        double bestTime = 0.0;
        
        for (double t = 0.0; t <= trajectory.getTotalTime(); t += 0.02) {
            Pose2d samplePose = trajectory.sampleAt(t, false).get().getPose();
            double dist = initialRobotPose.getTranslation().getDistance(samplePose.getTranslation());
            if (dist < minDistance) {
                minDistance = dist;
                bestTime = t;
            }
        }
        this.lastFoundTime = bestTime;
    }

    /**
     * Finds the closest point on the path, applies a tiny lookahead, 
     * and returns the exact Choreo SwerveSample for that moment.
     */
    public SwerveSample getLookaheadSample(Pose2d currentPose, Translation2d currentVelocity) {
        double totalTime = trajectory.getTotalTime();

        // 1. Sliding Window Search
        double searchStart = Math.max(0.0, lastFoundTime - 0.2); 
        double searchEnd = Math.min(totalTime, lastFoundTime + 0.5);

        double closestTime = searchStart;
        double minDistance = Double.MAX_VALUE;

        for (double t = searchStart; t <= searchEnd; t += 0.02) {
            Pose2d samplePose = trajectory.sampleAt(t, false).get().getPose();
            double dist = currentPose.getTranslation().getDistance(samplePose.getTranslation());
            
            if (dist < minDistance) {
                minDistance = dist;
                closestTime = t;
            }
        }

        lastFoundTime = closestTime;
        // 2. Dynamic Lookahead
        // Get the robot's current physical speed
        double currentSpeed = Math.hypot(currentVelocity.getX(), currentVelocity.getY());
        
        // At 0 m/s, lookahead is 0.1s (breaks friction).
        // At 4 m/s, lookahead shrinks to 0.02s (tight tracking).
        double lookaheadTime = Math.max(0.02, 0.1 - (currentSpeed * 0.02)); // Linear scaling between 0.05s and 0.012s based on speed
        
        double queryTime = Math.min(totalTime, closestTime + lookaheadTime);
        return trajectory.sampleAt(queryTime, false).get();
    }

    public double getCurrentPathTime() {
        return lastFoundTime;
    }
}