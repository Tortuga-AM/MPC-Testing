package frc.robot.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.GlobalConstants.TUNE_MODE;
import static frc.robot.Intake.IntakeConstants.*;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.team7525.subsystem.Subsystem;

/**
 * Intake is a singleton class that represents the intake subsystem of the robot.
 * It manages the state of the intake, including pivot and roller motor control, and provides methods for live tuning.
 */
public class Intake extends Subsystem<IntakeStates> implements AutoCloseable {

    private static Intake instance;
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs;
    private Angle pivotPosition;
    private LoggedNetworkNumber tunablePivotTargetAngle;
    private LoggedNetworkNumber tunableRollerTargetDutyCycle;

    /**
     * Returns the singleton instance of the Intake class.
     * If the instance does not exist, it creates a new one.
     *
     * @return The singleton instance of the Intake class.
     */
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        super("INTAKE", IntakeStates.IN_IDLE);
        io =
            switch (ROBOT_MODE) {
                case REAL -> new IntakeIOReal();
                case SIM -> new IntakeIOSim();
            };
        if (TUNE_MODE) {
            setupOverridesForLiveTuning();
        }
    }

    @Override
    public void runState() {
        inputs = io.updateInputs();
        if (TUNE_MODE) {
            runLiveTuning();
        } else {
            getState().runState(instance);
        }
        Logger.processInputs(SUBSYSTEM_NAME, inputs);
        Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().stateString);
    }

    /**
     * Sets up overrides for live tuning of the intake subsystem.
     * This method creates LoggedNetworkNumber instances for the pivot target angle and roller target duty cycle,
     * allowing them to be adjusted in real-time through the dashboard.
     */
    private void setupOverridesForLiveTuning() {
        tunablePivotTargetAngle = new LoggedNetworkNumber(SUBSYSTEM_NAME + "/PivotTargetAngle-TUNABLE (Degrees)", 0);
        tunableRollerTargetDutyCycle =
            new LoggedNetworkNumber(SUBSYSTEM_NAME + "/RollerTargetDutyCycle-TUNABLE (Percent)", 0);
        Logger.registerDashboardInput(tunablePivotTargetAngle);
        Logger.registerDashboardInput(tunableRollerTargetDutyCycle);
    }

    /**
     * Implements the default behavior of the intake subsystem.
     * This method sets the pivot motor position and roller motor duty cycle based on the current state of the intake.
     * It also logs the target pivot position in radians for monitoring purposes.
     */
    public void defaultBehavior() {
        io.setPivotMotorPosition(getState().intakeAngle);
        io.setRollerMotorDutyCycle(getState().intakeSpeed);
        Logger.recordOutput(SUBSYSTEM_NAME + "/TargetPivotPosition (Rads)", getState().intakeAngle.in(Radians));
    }

    /**
     * Implements the agitation behavior of the intake subsystem.
     * This method oscillates the pivot motor position between 0 and 90 degrees using a sine function,
     * while setting the roller motor duty cycle to the current state's intake speed.
     * It also logs the target pivot position in radians for monitoring purposes.
     */
    public void agitationBehavior() {
        // Follows a sin function to oscillate the roller motor speed
        // Maps oscilation to pivot position between 0 and 90 degrees to agitate intake
        pivotPosition = Degrees.of(45.0 + (45.0 * Math.sin(System.currentTimeMillis() / 1000.0 * 2 * Math.PI)));
        io.setPivotMotorPosition(pivotPosition);
        io.setRollerMotorDutyCycle(getState().intakeSpeed);
        Logger.recordOutput(SUBSYSTEM_NAME + "/TargetPivotPosition", pivotPosition.in(Radians));
    }

    /**
     * Runs the live tuning behavior of the intake subsystem.
     * This method sets the pivot motor position and roller motor duty cycle based on the tunable target values,
     * allowing for real-time adjustments through the dashboard.
     * It also logs the target pivot position in radians and the target roller duty cycle for monitoring purposes.
     */
    public void runLiveTuning() {
        io.setPivotMotorPosition(Degrees.of(tunablePivotTargetAngle.get()));
        io.setRollerMotorDutyCycle(Math.max(-1.0, Math.min(1.0, tunableRollerTargetDutyCycle.get())));
        Logger.recordOutput(
            SUBSYSTEM_NAME + "/TargetPivotPosition (Rads)",
            Degrees.of(tunablePivotTargetAngle.get()).in(Radians)
        );
        Logger.recordOutput(
            SUBSYSTEM_NAME + "/TargetRollerDutyCycle",
            Math.max(-1.0, Math.min(1.0, tunableRollerTargetDutyCycle.get()))
        );
        io.runLiveTuning();
    }

    @Override
    public void stateInit() {
        getState().onEnter(instance);
    }

    @Override
    public void stateExit() {
        getState().onExit(instance);
    }

    @Override
    public void close() throws Exception {
        io.close();
        instance = null;
    }
}
