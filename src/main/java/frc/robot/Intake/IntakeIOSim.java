package frc.robot.Intake;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Intake.IntakeConstants.*;

public class IntakeIOSim extends IntakeIOReal {
    private FlywheelSim rollerSim;
    private SingleJointedArmSim pivotSim;
    private TalonFXSimState leftPivotMotorSim;
    private TalonFXSimState rightPivotMotorSim;
    private TalonFXSimState leftRollerMotorSim;
    private TalonFXSimState rightRollerMotorSim;
    private IntakeIOInputsAutoLogged inputs;

    public IntakeIOSim() {
        super();
        inputs = new IntakeIOInputsAutoLogged();
        rollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                PhysicalConstants.ROLLER_MOI,
                PhysicalConstants.ROLLER_GEAR_RATIO
            ),
            DCMotor.getKrakenX60(2),
            PhysicalConstants.ROLLER_STD_DEV
        );
        pivotSim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getKrakenX60(2),
                PhysicalConstants.PIVOT_MOI,
                PhysicalConstants.PIVOT_GEAR_RATIO
            ),
            DCMotor.getKrakenX60(2),
            PhysicalConstants.PIVOT_GEAR_RATIO,
            PhysicalConstants.PIVOT_ARM_LENGTH,
            PhysicalConstants.PIVOT_MIN_ANGLE.in(Radians),
            PhysicalConstants.PIVOT_MAX_ANGLE.in(Radians),
            true,
            PhysicalConstants.PIVOT_MIN_ANGLE.in(Radians),
            PhysicalConstants.PIVOT_STD_DEV,
            PhysicalConstants.PIVOT_STD_DEV
        );
        leftPivotMotorSim = new TalonFXSimState(leftPivotMotor);
        rightPivotMotorSim = new TalonFXSimState(rightPivotMotor);
        leftRollerMotorSim = new TalonFXSimState(leftRollerMotor);
        rightRollerMotorSim = new TalonFXSimState(rightRollerMotor);
        leftPivotMotorSim.setMotorType(MotorType.KrakenX60);
        rightPivotMotorSim.setMotorType(MotorType.KrakenX60);
        leftRollerMotorSim.setMotorType(MotorType.KrakenX60);
        rightRollerMotorSim.setMotorType(MotorType.KrakenX60);
    }

    @Override
    public IntakeIOInputsAutoLogged updateInputs() {
        leftPivotMotorSim.setSupplyVoltage(Volts.of(12));
        rightPivotMotorSim.setSupplyVoltage(Volts.of(12));
        leftRollerMotorSim.setSupplyVoltage(Volts.of(12));
        rightRollerMotorSim.setSupplyVoltage(Volts.of(12));
        rollerSim.setInputVoltage(leftRollerMotorSim.getMotorVoltageMeasure().in(Volts));
        pivotSim.setInputVoltage(leftPivotMotorSim.getMotorVoltageMeasure().in(Volts));
        rollerSim.update(0.02);
        pivotSim.update(0.02);
        leftRollerMotorSim.setRotorVelocity(rollerSim.getAngularVelocity().times(PhysicalConstants.ROLLER_GEAR_RATIO));
        rightRollerMotorSim.setRotorVelocity(rollerSim.getAngularVelocity().times(PhysicalConstants.ROLLER_GEAR_RATIO));
        leftPivotMotorSim.setRawRotorPosition(Units.radiansToRotations(pivotSim.getAngleRads() * PhysicalConstants.PIVOT_GEAR_RATIO));
        rightPivotMotorSim.setRawRotorPosition(Units.radiansToRotations(pivotSim.getAngleRads() * PhysicalConstants.PIVOT_GEAR_RATIO));
        leftPivotMotorSim.setRotorVelocity(Units.radiansToRotations(pivotSim.getVelocityRadPerSec() * PhysicalConstants.PIVOT_GEAR_RATIO));
        rightPivotMotorSim.setRotorVelocity(Units.radiansToRotations(pivotSim.getVelocityRadPerSec() * PhysicalConstants.PIVOT_GEAR_RATIO));

        // Update the inputs with simulated values
        inputs.leftPivotMotorCurrent = Amps.of(leftPivotMotorSim.getSupplyCurrent());
        inputs.rightPivotMotorCurrent = Amps.of(rightPivotMotorSim.getSupplyCurrent());
        inputs.leftRollerMotorCurrent = Amps.of(leftRollerMotorSim.getSupplyCurrent());
        inputs.rightRollerMotorCurrent = Amps.of(rightRollerMotorSim.getSupplyCurrent());
        inputs.leftPivotMotorVoltage = leftPivotMotorSim.getMotorVoltageMeasure();
        inputs.rightPivotMotorVoltage = rightPivotMotorSim.getMotorVoltageMeasure();
        inputs.leftRollerMotorVoltage = leftRollerMotorSim.getMotorVoltageMeasure();
        inputs.rightRollerMotorVoltage = rightRollerMotorSim.getMotorVoltageMeasure();
        inputs.leftPivotMotorAngle = Radians.of(pivotSim.getAngleRads()); // Not implemented in simulation
        inputs.rightPivotMotorAngle = Radians.of(pivotSim.getAngleRads()); // Not implemented in simulation
        inputs.leftPivotMotorVelocity = RadiansPerSecond.of(pivotSim.getVelocityRadPerSec()); // Not implemented in simulation
        inputs.rightPivotMotorVelocity = RadiansPerSecond.of(pivotSim.getVelocityRadPerSec()); // Not implemented in simulation
        inputs.leftRollerMotorAngle = edu.wpi.first.units.Units.Degrees.of(0); // Not implemented in simulation
        inputs.rightRollerMotorAngle = edu.wpi.first.units.Units.Degrees.of(0); // Not implemented in simulation
        inputs.leftRollerMotorVelocity = rollerSim.getAngularVelocity();
        inputs.rightRollerMotorVelocity = rollerSim.getAngularVelocity();
        return inputs;
    }

}
