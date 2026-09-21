package frc.robot.Manager;

import frc.robot.Intake.IntakeStates;
import org.team7525.subsystem.SubsystemStates;

/**
 * ManagerStates is an enum that defines the different states of the Manager subsystem.
 * Each state is associated with a corresponding IntakeState, which represents the state of the Intake subsystem.
 */
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
