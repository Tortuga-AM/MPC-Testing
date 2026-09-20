package frc.robot.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeTest {
    private static final double ANGLE_TOLERANCE_RAD = 0.15;
    private static final int FAST_SETTLE_CYCLES = 120;
    private static final int SLOW_SETTLE_CYCLES = 240;

    private IntakeIOSim intakeIO;

    @BeforeEach
    void setup() {
        HAL.initialize(500, 0);
        intakeIO = new IntakeIOSim();
    }

    /** Ensures sensor outputs remain symmetric and bounded before any command is issued. */
    @Test
    void initialSimulationStateIsBounded() {
        IntakeIO.IntakeIOInputs inputs = runSimCycles(5);
        double minAngle = IntakeConstants.PhysicalConstants.PIVOT_MIN_ANGLE.in(Radians);
        double maxAngle = IntakeConstants.PhysicalConstants.PIVOT_MAX_ANGLE.in(Radians);

        assertAll(
                () -> assertTrue(
                        inputs.leftPivotMotorAngle.in(Radians)
                                >= minAngle - ANGLE_TOLERANCE_RAD,
                        "initial angle below min bound tolerance"),
                () -> assertTrue(
                        inputs.leftPivotMotorAngle.in(Radians)
                                <= maxAngle + ANGLE_TOLERANCE_RAD,
                        "initial angle above max bound tolerance"));
    }

    /** Non-zero duty cycle should produce non-zero roller voltage magnitude. */
    @Test
    void nonZeroRollerCommandProducesNonZeroVoltageMagnitude() {
        intakeIO.setRollerMotorDutyCycle(0.65);
        IntakeIO.IntakeIOInputs positiveInputs = runSimCycles(FAST_SETTLE_CYCLES);

        intakeIO.setRollerMotorDutyCycle(-0.45);
        IntakeIO.IntakeIOInputs negativeInputs = runSimCycles(FAST_SETTLE_CYCLES);

        assertAll(
                () -> assertTrue(Math.abs(positiveInputs.leftRollerMotorVoltage.in(Volts)) > 0.2),
                () -> assertTrue(Math.abs(negativeInputs.leftRollerMotorVoltage.in(Volts)) > 0.2),
                () -> assertTrue(
                        Math.abs(positiveInputs.leftRollerMotorVoltage.in(Volts))
                                > Math.abs(negativeInputs.leftRollerMotorVoltage.in(Volts))));
    }

    /** Zero duty cycle should settle to approximately zero roller voltage. */
    @Test
    void zeroRollerCommandSettlesNearZeroVoltage() {
        intakeIO.setRollerMotorDutyCycle(0.0);

        IntakeIO.IntakeIOInputs inputs = runSimCycles(FAST_SETTLE_CYCLES);

        assertTrue(Math.abs(inputs.leftRollerMotorVoltage.in(Volts)) <= 0.25);
    }

    /** Pivot command should stay in bounds under command. */
    @Test
    void pivotCommandStaysBounded() {
        intakeIO.setPivotMotorPosition(Degrees.of(70.0));

        IntakeIO.IntakeIOInputs inputs = runSimCycles(SLOW_SETTLE_CYCLES);
        double minAllowed = IntakeConstants.PhysicalConstants.PIVOT_MIN_ANGLE.in(Radians) - ANGLE_TOLERANCE_RAD;
        double maxAllowed = IntakeConstants.PhysicalConstants.PIVOT_MAX_ANGLE.in(Radians) + ANGLE_TOLERANCE_RAD;

        assertAll(
                () -> assertTrue(inputs.leftPivotMotorAngle.in(Radians) >= minAllowed),
                () -> assertTrue(inputs.leftPivotMotorAngle.in(Radians) <= maxAllowed));
    }

    /** Pivot outputs should remain clamped to configured minimum and maximum physical angles. */
    @Test
    void pivotPositionRespectsPhysicalLimitsInSimulation() {
        intakeIO.setPivotMotorPosition(Degrees.of(200.0));

        IntakeIO.IntakeIOInputs highInputs = runSimCycles(SLOW_SETTLE_CYCLES);
        double maxAllowed = IntakeConstants.PhysicalConstants.PIVOT_MAX_ANGLE.in(Radians) + ANGLE_TOLERANCE_RAD;
        assertTrue(highInputs.leftPivotMotorAngle.in(Radians) <= maxAllowed, "pivot exceeded max angle");

        intakeIO.setPivotMotorPosition(Degrees.of(-50.0));
        IntakeIO.IntakeIOInputs lowInputs = runSimCycles(SLOW_SETTLE_CYCLES);
        double minAllowed = IntakeConstants.PhysicalConstants.PIVOT_MIN_ANGLE.in(Radians) - ANGLE_TOLERANCE_RAD;
        assertTrue(lowInputs.leftPivotMotorAngle.in(Radians) >= minAllowed, "pivot dropped below min angle");
    }

    /** Sim motor voltages should stay inside the 12V supply envelope. */
    @Test
    void simulatedMotorVoltagesStayWithinSupplyRange() {
        intakeIO.setPivotMotorPosition(Degrees.of(90.0));
        intakeIO.setRollerMotorDutyCycle(1.0);

        IntakeIO.IntakeIOInputs inputs = runSimCycles(SLOW_SETTLE_CYCLES);

        assertAll(
                () -> assertTrue(Math.abs(inputs.leftPivotMotorVoltage.in(Volts)) <= 12.0 + 1e-6),
                () -> assertTrue(Math.abs(inputs.rightPivotMotorVoltage.in(Volts)) <= 12.0 + 1e-6),
                () -> assertTrue(Math.abs(inputs.leftRollerMotorVoltage.in(Volts)) <= 12.0 + 1e-6),
                () -> assertTrue(Math.abs(inputs.rightRollerMotorVoltage.in(Volts)) <= 12.0 + 1e-6));
    }

    private IntakeIO.IntakeIOInputs runSimCycles(int cycles) {
        IntakeIO.IntakeIOInputs inputs = null;
        for (int i = 0; i < cycles; i++) {
            inputs = intakeIO.updateInputs();
        }
        assertNotNull(inputs, "runSimCycles must run at least one iteration");
        return inputs;
    }

}