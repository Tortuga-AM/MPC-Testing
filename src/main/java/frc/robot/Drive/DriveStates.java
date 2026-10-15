package frc.robot.Drive;

import org.team7525.subsystem.SubsystemStates;

public enum DriveStates implements SubsystemStates {
    MANUAL("Manual"),
    AUTO_ALIGN("Auto Align"),
    BRAKE("Brake");

    private final String stateString;

    DriveStates(String stateString) {
        this.stateString = stateString;
    }

    @Override
    public String getStateString() {
        return stateString;
    }
}