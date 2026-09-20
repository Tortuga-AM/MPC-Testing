package frc.robot.Intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.team7525.subsystem.Subsystem;
import edu.wpi.first.units.measure.Angle;
import static frc.robot.Intake.IntakeConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.GlobalConstants.TUNE_MODE;

public class Intake extends Subsystem<IntakeStates> {
    private static Intake instance;
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs;
    private Angle pivotPosition;
    private LoggedNetworkNumber tunablePivotTargetAngle;
    private LoggedNetworkNumber tunableRollerTargetDutyCycle;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        super("INTAKE", IntakeStates.IN_IDLE);
        io = switch (ROBOT_MODE) {
            case REAL -> new IntakeIOReal();
            case SIM -> new IntakeIOSim();
        };
        if (TUNE_MODE) {setupOverridesForLiveTuning();}
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

    private void setupOverridesForLiveTuning() {
        tunablePivotTargetAngle = new LoggedNetworkNumber(SUBSYSTEM_NAME + "/PivotTargetAngle-TUNABLE (Degrees)", 0);
        tunableRollerTargetDutyCycle = new LoggedNetworkNumber(SUBSYSTEM_NAME + "/RollerTargetDutyCycle-TUNABLE (Percent)", 0);
        Logger.registerDashboardInput(tunablePivotTargetAngle);
        Logger.registerDashboardInput(tunableRollerTargetDutyCycle);
    }

    public void defaultBehavior() {
        io.setPivotMotorPosition(getState().intakeAngle);
        io.setRollerMotorDutyCycle(getState().intakeSpeed);
        Logger.recordOutput(SUBSYSTEM_NAME + "/TargetPivotPosition (Rads)", getState().intakeAngle.in(Radians));
    }

    public void agitationBehavior() {
        // Follows a sin function to oscillate the roller motor speed
        // Maps oscilation to pivot position between 0 and 90 degrees to agitate the intake
        pivotPosition = Degrees.of(45.0 + (45.0 * Math.sin(System.currentTimeMillis() / 1000.0 * 2 * Math.PI))); // Oscillates between 0 and 90 degrees
        io.setPivotMotorPosition(pivotPosition);
        io.setRollerMotorDutyCycle(getState().intakeSpeed);
        Logger.recordOutput(SUBSYSTEM_NAME + "/TargetPivotPosition", pivotPosition.in(Radians));
    }

    public void runLiveTuning() {
        io.setPivotMotorPosition(Degrees.of(tunablePivotTargetAngle.get()));
        io.setRollerMotorDutyCycle(Math.max(-1.0, Math.min(1.0, tunableRollerTargetDutyCycle.get())));
        Logger.recordOutput(SUBSYSTEM_NAME + "/TargetPivotPosition (Rads)", Degrees.of(tunablePivotTargetAngle.get()).in(Radians));
        Logger.recordOutput(SUBSYSTEM_NAME + "/TargetRollerDutyCycle", Math.max(-1.0, Math.min(1.0, tunableRollerTargetDutyCycle.get())));
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
}
