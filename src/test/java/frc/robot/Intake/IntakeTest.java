package frc.robot.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeTest {
    private static final double ANGLE_TOLERANCE_RAD = 0.15;
    private static final double VELOCITY_TOLERANCE_RAD_PER_SEC = 1e-2;
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
    void initialSimulationStateIsSymmetricAndWithinPhysicalLimits() {
        IntakeIO.IntakeIOInputs inputs = runSimCycles(5);
        double minAngle = IntakeConstants.PhysicalConstants.PIVOT_MIN_ANGLE.in(Radians);
        double maxAngle = IntakeConstants.PhysicalConstants.PIVOT_MAX_ANGLE.in(Radians);

        assertAll(
                () -> assertApproximately(
                        inputs.leftPivotMotorAngle.in(Radians),
                        inputs.rightPivotMotorAngle.in(Radians),
                        ANGLE_TOLERANCE_RAD,
                        "pivot angles should match"),
                () -> assertApproximately(
                        inputs.leftPivotMotorVelocity.in(RadiansPerSecond),
                        inputs.rightPivotMotorVelocity.in(RadiansPerSecond),
                        VELOCITY_TOLERANCE_RAD_PER_SEC,
                        "pivot velocities should match"),
                () -> assertApproximately(
                        inputs.leftRollerMotorVelocity.in(RadiansPerSecond),
                        inputs.rightRollerMotorVelocity.in(RadiansPerSecond),
                        VELOCITY_TOLERANCE_RAD_PER_SEC,
                        "roller velocities should match"),
                () -> assertTrue(
                        inputs.leftPivotMotorAngle.in(Radians)
                                >= minAngle - ANGLE_TOLERANCE_RAD,
                        "initial angle below min bound tolerance"),
                () -> assertTrue(
                        inputs.leftPivotMotorAngle.in(Radians)
                                <= maxAngle + ANGLE_TOLERANCE_RAD,
                        "initial angle above max bound tolerance"));
    }

    /** Positive duty cycle should produce positive commanded roller voltage. */
    @Test
    void positiveRollerCommandProducesPositiveVoltage() {
        intakeIO.setRollerMotorDutyCycle(0.65);

        IntakeIO.IntakeIOInputs inputs = runSimCycles(FAST_SETTLE_CYCLES);

        assertAll(
                () -> assertTrue(inputs.leftRollerMotorVoltage.in(Volts) > 0.0),
                () -> assertApproximately(
                        Math.abs(inputs.leftRollerMotorVoltage.in(Volts)),
                        Math.abs(inputs.rightRollerMotorVoltage.in(Volts)),
                        0.2,
                        "left/right roller voltage magnitudes should match"));
    }

    /** Negative duty cycle should produce negative commanded roller voltage. */
    @Test
    void negativeRollerCommandProducesNegativeVoltage() {
        intakeIO.setRollerMotorDutyCycle(-0.45);

        IntakeIO.IntakeIOInputs inputs = runSimCycles(FAST_SETTLE_CYCLES);

        assertAll(
                () -> assertTrue(inputs.leftRollerMotorVoltage.in(Volts) < 0.0),
                () -> assertApproximately(
                        Math.abs(inputs.leftRollerMotorVoltage.in(Volts)),
                        Math.abs(inputs.rightRollerMotorVoltage.in(Volts)),
                        0.2,
                        "left/right roller voltage magnitudes should match"));
    }

    /** Pivot output channels should stay synchronized and in bounds under command. */
    @Test
    void pivotCommandKeepsOutputsSynchronizedAndBounded() {
        intakeIO.setPivotMotorPosition(Degrees.of(70.0));

        IntakeIO.IntakeIOInputs inputs = runSimCycles(SLOW_SETTLE_CYCLES);
        double minAllowed = IntakeConstants.PhysicalConstants.PIVOT_MIN_ANGLE.in(Radians) - ANGLE_TOLERANCE_RAD;
        double maxAllowed = IntakeConstants.PhysicalConstants.PIVOT_MAX_ANGLE.in(Radians) + ANGLE_TOLERANCE_RAD;

        assertAll(
                () -> assertApproximately(
                        Math.abs(inputs.leftPivotMotorAngle.in(Radians)),
                        Math.abs(inputs.rightPivotMotorAngle.in(Radians)),
                        ANGLE_TOLERANCE_RAD,
                        "left/right pivot angle magnitudes should match"),
                () -> assertApproximately(
                        Math.abs(inputs.leftPivotMotorVelocity.in(RadiansPerSecond)),
                        Math.abs(inputs.rightPivotMotorVelocity.in(RadiansPerSecond)),
                        VELOCITY_TOLERANCE_RAD_PER_SEC,
                        "left/right pivot velocity magnitudes should match"),
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

    private static void assertApproximately(double actual, double expected, double tolerance, String message) {
        assertTrue(Math.abs(actual - expected) <= tolerance, message + " (actual=" + actual + ", expected=" + expected + ")");
    }
}