package frc.robot.Drive;

import org.team7525.subsystem.SubsystemStates;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum DriveStates implements SubsystemStates {
    MANUAL("Manual", Pose2d.kZero, false);

    private final String stateString;
    private final boolean pathfindingEnabled;
    private Pose2d pose;

    DriveStates(String stateString, Pose2d pose, boolean pathfindingEnabled) {
        this.stateString = stateString;
        this.pose = pose;
        this.pathfindingEnabled = pathfindingEnabled;
    }

    @Override
    public String getStateString() {
        return stateString;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public boolean isPathfindingEnabled() {
        return pathfindingEnabled;
    }
}