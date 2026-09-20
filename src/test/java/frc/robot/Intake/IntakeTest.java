package frc.robot.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class IntakeTest {
    private static final double ANGLE_TOLERANCE_RAD = 0.35;
    private static final double VELOCITY_TOLERANCE_RAD_PER_SEC = 0.2;

    private IntakeIOSim intakeIO;

    @BeforeAll
    void setup() {
        HAL.initialize(500, 0);
        intakeIO = new IntakeIOSim();
    }

    @Test
    void rollerVelocityTracksPositiveDutyCycleInSimulation() {
        intakeIO.setRollerMotorDutyCycle(0.65);

        IntakeIOInputsAutoLogged inputs = runSimCycles(100);

        assertTrue(inputs.leftRollerMotorVelocity.in(RadiansPerSecond) > 0.0);
        assertEquals(
                inputs.leftRollerMotorVelocity.in(RadiansPerSecond),
                inputs.rightRollerMotorVelocity.in(RadiansPerSecond),
                VELOCITY_TOLERANCE_RAD_PER_SEC);
    }

    @Test
    void rollerVelocityTracksNegativeDutyCycleInSimulation() {
        intakeIO.setRollerMotorDutyCycle(-0.45);

        IntakeIOInputsAutoLogged inputs = runSimCycles(100);

        assertTrue(inputs.leftRollerMotorVelocity.in(RadiansPerSecond) < 0.0);
        assertEquals(
                inputs.leftRollerMotorVelocity.in(RadiansPerSecond),
                inputs.rightRollerMotorVelocity.in(RadiansPerSecond),
                VELOCITY_TOLERANCE_RAD_PER_SEC);
    }

    @Test
    void pivotPositionConvergesToSetpointInSimulation() {
        double targetRadians = Degrees.of(70.0).in(Radians);
        intakeIO.setPivotMotorPosition(Degrees.of(70.0));

        IntakeIOInputsAutoLogged inputs = runSimCycles(180);

        assertEquals(targetRadians, inputs.leftPivotMotorAngle.in(Radians), ANGLE_TOLERANCE_RAD);
        assertEquals(
                inputs.leftPivotMotorAngle.in(Radians),
                inputs.rightPivotMotorAngle.in(Radians),
                ANGLE_TOLERANCE_RAD);
    }

    @Test
    void pivotPositionRespectsPhysicalLimitsInSimulation() {
        intakeIO.setPivotMotorPosition(Degrees.of(200.0));

        IntakeIOInputsAutoLogged highInputs = runSimCycles(220);
        double maxAllowed =
                IntakeConstants.PhysicalConstants.PIVOT_MAX_ANGLE.in(Radians) + ANGLE_TOLERANCE_RAD;
        assertTrue(highInputs.leftPivotMotorAngle.in(Radians) <= maxAllowed);

        intakeIO.setPivotMotorPosition(Degrees.of(-50.0));
        IntakeIOInputsAutoLogged lowInputs = runSimCycles(220);
        double minAllowed =
                IntakeConstants.PhysicalConstants.PIVOT_MIN_ANGLE.in(Radians) - ANGLE_TOLERANCE_RAD;
        assertTrue(lowInputs.leftPivotMotorAngle.in(Radians) >= minAllowed);
    }

    private IntakeIOInputsAutoLogged runSimCycles(int cycles) {
        IntakeIOInputsAutoLogged inputs = null;
        for (int i = 0; i < cycles; i++) {
            inputs = intakeIO.updateInputs();
        }
        return inputs;
    }
}