package frc.robot.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/**
 * IntakeIO is an interface that defines the input/output operations for the intake subsystem.
 * It provides methods to update inputs, set motor positions and duty cycles, and run live tuning.
 */
public interface IntakeIO extends AutoCloseable {
    /**
     * IntakeIOInputs is a class that holds the input values for the intake subsystem.
     * It includes motor current and voltage values, pivot and roller encoder values, and beam break sensor status.
     */
    @AutoLog
    public class IntakeIOInputs {

        // -- Motor Current Values -- //
        public Current leftPivotMotorCurrent = Amps.of(0.0);
        public Current rightPivotMotorCurrent = Amps.of(0.0);
        public Current leftRollerMotorCurrent = Amps.of(0.0);
        public Current rightRollerMotorCurrent = Amps.of(0.0);

        // -- Motor Voltage Values -- //
        public Voltage leftPivotMotorVoltage = Volts.of(0.0);
        public Voltage rightPivotMotorVoltage = Volts.of(0.0);
        public Voltage leftRollerMotorVoltage = Volts.of(0.0);
        public Voltage rightRollerMotorVoltage = Volts.of(0.0);

        // -- Pivot Encoder Values -- //
        public Angle leftPivotMotorAngle = Degrees.of(0.0);
        public Angle rightPivotMotorAngle = Degrees.of(0.0);
        public AngularVelocity leftPivotMotorVelocity = DegreesPerSecond.of(0.0);
        public AngularVelocity rightPivotMotorVelocity = DegreesPerSecond.of(0.0);

        // -- Roller Encoder Values -- //
        public Angle leftRollerMotorAngle = Degrees.of(0.0);
        public Angle rightRollerMotorAngle = Degrees.of(0.0);
        public AngularVelocity leftRollerMotorVelocity = RotationsPerSecond.of(0.0);
        public AngularVelocity rightRollerMotorVelocity = RotationsPerSecond.of(0.0);

        // -- Beam Break Sensor Values -- //
        public boolean beamBreakSensor = false;
    }

    public abstract IntakeIOInputsAutoLogged updateInputs();

    public abstract void setPivotMotorPosition(Angle position);

    public abstract void setRollerMotorDutyCycle(double dutyCycle);

    public abstract void runLiveTuning();
}
