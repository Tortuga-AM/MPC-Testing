package frc.robot.Intake;

import static edu.wpi.first.units.Units.Degrees;
import java.util.function.Consumer;
import edu.wpi.first.units.measure.Angle;
import org.team7525.subsystem.SubsystemStates;

public enum IntakeStates implements SubsystemStates {
    IN_IDLE("In Idle", Degrees.of(0), 0.0),
    OUT_IDLE("Out Idle", Degrees.of(90), 0.0),
    INTAKING("Intaking", Degrees.of(90), 1.0),
    AGITATING(
        "Agitating", 
        Degrees.of(45), 
        0.5, 
        subsystem -> System.out.println("Starting agitator!"), // onEnter
        subsystem -> subsystem.agitationBehavior(), // runState
        subsystem -> System.out.println("Ending agitator!")    // onExit
    ),
    // -- BACKUP/FIXING STATES -- //
    OUTTAKING("Outtaking", Degrees.of(90), -1.0),
    ZEROING("Zeroing", Degrees.of(0), 0.0);

    public final String stateString;
    public final Angle intakeAngle;
    public final Double intakeSpeed;
    private final Consumer<Intake> onEnter;
    private final Consumer<Intake> runState;
    private final Consumer<Intake> onExit;

    IntakeStates(String stateString, Angle intakeAngle, Double intakeSpeed) {
        this.stateString = stateString;
        this.intakeAngle = intakeAngle;
        this.intakeSpeed = intakeSpeed;
        this.onEnter = subsystem -> {};
        this.onExit = subsystem -> {};
        this.runState = subsystem -> subsystem.defaultBehavior();
    }

    IntakeStates(String stateString, Angle intakeAngle, Double intakeSpeed, 
                 Consumer<Intake> onEnter, 
                 Consumer<Intake> runState, 
                 Consumer<Intake> onExit) {
        this.stateString = stateString;
        this.intakeAngle = intakeAngle;
        this.intakeSpeed = intakeSpeed;

        // Custom lambdas
        this.onEnter = onEnter;
        this.runState = runState;
        this.onExit = onExit;
    }

    public void onEnter(Intake subsystem) { onEnter.accept(subsystem); }
    public void runState(Intake subsystem) { runState.accept(subsystem); }
    public void onExit(Intake subsystem) { onExit.accept(subsystem); }
}
