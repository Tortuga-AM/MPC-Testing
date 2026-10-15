package frc.robot.Drive.AutoAlign;

import org.team7525.subsystem.SubsystemStates;

public enum AutoAlignStates implements SubsystemStates {
    IDLE("Idle"),
    PLANNING("Planning"),
    FOLLOWING_TRAJECTORY("Following Trajectory"),
    REPLANNING("Replanning"),
    AT_TARGET("At Target"),
    FAILED("Failed"),
    CANCELLED("Cancelled");

    private final String stateString;

    AutoAlignStates(String stateString) {
        this.stateString = stateString;
    }

    @Override
    public String getStateString() {
        return stateString;
    }
}