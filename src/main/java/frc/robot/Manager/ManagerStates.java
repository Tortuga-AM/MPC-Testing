package frc.robot.Manager;

import org.team7525.subsystem.SubsystemStates;

import frc.robot.Intake.IntakeStates;

public enum ManagerStates implements SubsystemStates {
    IN_IDLE(IntakeStates.IN_IDLE),
    OUT_IDLE(IntakeStates.OUT_IDLE),
    INTAKING(IntakeStates.INTAKING),
    AGITATING(IntakeStates.AGITATING),
    OUTTAKING(IntakeStates.OUTTAKING),
    ZEROING(IntakeStates.ZEROING);

    public final IntakeStates intakeState;

    ManagerStates(IntakeStates intakeState) {
        this.intakeState = intakeState;
    }
}
