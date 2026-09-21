package frc.robot.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.GlobalConstants.ROBOT_MODE;

import edu.wpi.first.units.measure.Angle;
import org.team7525.controlConstants.PIDConstants;

/**
 * A class to hold all of the constants for the intake subsystem.
 */
public class IntakeConstants {

    public static final String SUBSYSTEM_NAME = "Intake";

    public static final PIDConstants PIVOT_PID_CONTROLLER =
        switch (ROBOT_MODE) {
            case REAL -> new PIDConstants(0.0, 0.0, 0.0);
            case SIM -> new PIDConstants(150, 0.0, 2);
        };

    public static final double INTAKE_ROLLER_DUTY_CYCLE = 1.0;
    public static final double OUTTAKE_ROLLER_DUTY_CYCLE = -1.0;
    public static final double AGITATION_ROLLER_DUTY_CYCLE = 0.5;
    public static final double IDLE_ROLLER_DUTY_CYCLE = 0.0;

    /**
     * A class to hold all of the physical constants for the intake subsystem.
     */
    public static final class PhysicalConstants {

        public static final double PIVOT_GEAR_RATIO = 100.0 / 1.0; // Gear ratio of the pivot motor (input/output)
        public static final double ROLLER_GEAR_RATIO = 1.0 / 1.0; // Gear ratio of the roller motor (output/input)
        public static final double ROLLER_MOI = 0.003; // Moment of inertia of the roller in kg*m^2
        public static final double PIVOT_MOI = 0.12; // Moment of inertia of the pivot in kg*m^2
        public static final double PIVOT_ARM_LENGTH = 0.5; // Length of the pivot arm in meters
        public static final double ROLLER_STD_DEV = 0.05; // Standard deviation of the roller position sensor in meters
        public static final double PIVOT_STD_DEV = 0.005; // Standard deviation of the pivot position sensor in meters
        public static final Angle PIVOT_MIN_ANGLE = Degrees.of(0); // Minimum angle of the pivot in degrees
        public static final Angle PIVOT_MAX_ANGLE = Degrees.of(90); // Maximum angle of the pivot in degrees
    }
}
