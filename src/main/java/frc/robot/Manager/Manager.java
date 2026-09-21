package frc.robot.Manager;

import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;

import frc.robot.Intake.Intake;
import org.team7525.subsystem.Subsystem;

/**
 * The Manager class is a singleton that manages the state of the robot's intake system.
 * It extends the Subsystem class and defines various states for managing the intake,
 * including idle, intaking, outtaking, agitating, and zeroing.
 */
public class Manager extends Subsystem<ManagerStates> {

    public static final Manager INSTANCE = new Manager();
    private Intake intake;

    public static Manager getInstance() {
        return INSTANCE;
    }

    private Manager() {
        super("MANAGER", ManagerStates.IN_IDLE);
        intake = Intake.getInstance();
        // drive = Drive.getInstance();
        addTrigger(ManagerStates.IN_IDLE, ManagerStates.INTAKING, DRIVER_CONTROLLER::getAButtonPressed);
        addTrigger(ManagerStates.IN_IDLE, ManagerStates.OUT_IDLE, DRIVER_CONTROLLER::getBButtonPressed);
        addTrigger(ManagerStates.IN_IDLE, ManagerStates.AGITATING, DRIVER_CONTROLLER::getXButtonPressed);
        addTrigger(ManagerStates.IN_IDLE, ManagerStates.ZEROING, DRIVER_CONTROLLER::getYButtonPressed);
        addTrigger(ManagerStates.INTAKING, ManagerStates.IN_IDLE, DRIVER_CONTROLLER::getAButtonReleased);
        addTrigger(ManagerStates.OUT_IDLE, ManagerStates.IN_IDLE, DRIVER_CONTROLLER::getBButtonReleased);
        addTrigger(ManagerStates.AGITATING, ManagerStates.IN_IDLE, DRIVER_CONTROLLER::getXButtonReleased);
        addTrigger(ManagerStates.ZEROING, ManagerStates.IN_IDLE, DRIVER_CONTROLLER::getYButtonReleased);
        addRunnableTrigger(
            () -> setState(ManagerStates.IN_IDLE),
            () ->
                DRIVER_CONTROLLER.getAButton() &&
                DRIVER_CONTROLLER.getBButton() &&
                DRIVER_CONTROLLER.getXButton() &&
                DRIVER_CONTROLLER.getYButton()
        );
    }

    @Override
    public void runState() {
        intake.setState(getState().intakeState);
        intake.periodic();
    }
}
