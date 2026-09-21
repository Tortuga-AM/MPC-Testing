package frc.robot.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeTest {

    Intake intake;

    @BeforeEach
    void setUp() {
        assert HAL.initialize(500, 0);
        intake = Intake.getInstance();
        intake.setState(IntakeStates.IN_IDLE);
    }

    private void runSimulation(int loops) {
        for (int i = 0; i < loops; i++) {
            intake.periodic();
            // Advance WPILib simulation time by the standard 20ms robot loop
            SimHooks.stepTiming(0.02);
        }
    }

    @SuppressWarnings("PMD.SignatureDeclareThrowsException")
    @AfterEach
    void shutdown() throws Exception {
        intake.close();
    }

    @Test
    void testIntakeInitialization() {
        assertEquals(IntakeStates.IN_IDLE, intake.getState());
    }

    @Test
    void testStateTransition() {
        intake.setState(IntakeStates.INTAKING);
        assertEquals(IntakeStates.INTAKING, intake.getState());
    }

    @Test
    void testInitialPositionAndSpeed() {
        intake.setState(IntakeStates.IN_IDLE);
        runSimulation(500);
        assertEquals(IntakeStates.IN_IDLE.intakeAngle.in(Degrees), intake.getState().intakeAngle.in(Degrees), 3);
        assertEquals(IntakeStates.IN_IDLE.intakeSpeed, intake.getState().intakeSpeed, 1);
    }

    @Test
    void testIntakingPositionAndSpeed() {
        intake.setState(IntakeStates.INTAKING);
        runSimulation(500);
        assertEquals(IntakeStates.INTAKING.intakeAngle.in(Degrees), intake.getState().intakeAngle.in(Degrees), 3);
        assertEquals(IntakeStates.INTAKING.intakeSpeed, intake.getState().intakeSpeed, 1);
    }
}
