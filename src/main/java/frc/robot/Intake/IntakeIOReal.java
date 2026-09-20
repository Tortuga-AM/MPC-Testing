package frc.robot.Intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import static frc.robot.GlobalConstants.TUNE_MODE;
import static frc.robot.Intake.IntakeConstants.PIVOT_PID_CONTROLLER;

public class IntakeIOReal implements IntakeIO {
    private IntakeIOInputsAutoLogged inputs;
    protected TalonFX leftPivotMotor;
    protected TalonFX rightPivotMotor;
    protected TalonFX leftRollerMotor;
    protected TalonFX rightRollerMotor;
    protected LoggedNetworkNumber kP;
    protected LoggedNetworkNumber kI;
    protected LoggedNetworkNumber kD;


    public IntakeIOReal() {
        inputs = new IntakeIOInputsAutoLogged();
        leftPivotMotor = new TalonFX(1);
        rightPivotMotor = new TalonFX(2);
        leftRollerMotor = new TalonFX(3);
        rightRollerMotor = new TalonFX(4);

        var pivotConfigs = new TalonFXConfiguration();
        pivotConfigs.Slot0.kP = PIVOT_PID_CONTROLLER.kP;
        pivotConfigs.Slot0.kI = PIVOT_PID_CONTROLLER.kI;
        pivotConfigs.Slot0.kD = PIVOT_PID_CONTROLLER.kD;
        pivotConfigs.Feedback.SensorToMechanismRatio = IntakeConstants.PhysicalConstants.PIVOT_GEAR_RATIO;

        leftPivotMotor.getConfigurator().apply(pivotConfigs);
        rightPivotMotor.getConfigurator().apply(pivotConfigs);

        var rollerConfigs = new TalonFXConfiguration();
        leftRollerMotor.getConfigurator().apply(rollerConfigs);
        rightRollerMotor.getConfigurator().apply(rollerConfigs);
        if (TUNE_MODE) {setupOverridesForLiveTuning();}
    }

    public void setupOverridesForLiveTuning() {
        kP = new LoggedNetworkNumber("Intake/Pivot kP-TUNABLE", PIVOT_PID_CONTROLLER.kP);
        kI = new LoggedNetworkNumber("Intake/Pivot kI-TUNABLE", PIVOT_PID_CONTROLLER.kI);
        kD = new LoggedNetworkNumber("Intake/Pivot kD-TUNABLE", PIVOT_PID_CONTROLLER.kD);
    }

    @Override
    public IntakeIOInputsAutoLogged updateInputs() {
        inputs.leftPivotMotorCurrent = leftPivotMotor.getSupplyCurrent().getValue();
        inputs.rightPivotMotorCurrent = rightPivotMotor.getSupplyCurrent().getValue();
        inputs.leftRollerMotorCurrent = leftRollerMotor.getSupplyCurrent().getValue();
        inputs.rightRollerMotorCurrent = rightRollerMotor.getSupplyCurrent().getValue();
        inputs.leftPivotMotorVoltage = leftPivotMotor.getSupplyVoltage().getValue();
        inputs.rightPivotMotorVoltage = rightPivotMotor.getSupplyVoltage().getValue();
        inputs.leftRollerMotorVoltage = leftRollerMotor.getSupplyVoltage().getValue();
        inputs.rightRollerMotorVoltage = rightRollerMotor.getSupplyVoltage().getValue();
        inputs.leftPivotMotorAngle = leftPivotMotor.getPosition().getValue();
        inputs.rightPivotMotorAngle = rightPivotMotor.getPosition().getValue();
        inputs.leftPivotMotorVelocity = leftPivotMotor.getVelocity().getValue();
        inputs.rightPivotMotorVelocity = rightPivotMotor.getVelocity().getValue();
        inputs.leftRollerMotorAngle = leftRollerMotor.getPosition().getValue();
        inputs.rightRollerMotorAngle = rightRollerMotor.getPosition().getValue();
        inputs.leftRollerMotorVelocity = leftRollerMotor.getVelocity().getValue();
        inputs.rightRollerMotorVelocity = rightRollerMotor.getVelocity().getValue();
        return inputs;
    }

    @Override
    public void setPivotMotorPosition(Angle position) {
        leftPivotMotor.setControl(new PositionVoltage(position));
        rightPivotMotor.setControl(new Follower(leftPivotMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void setRollerMotorDutyCycle(double dutyCycle) {
        leftRollerMotor.setControl(new DutyCycleOut(dutyCycle));
        rightRollerMotor.setControl(new Follower(leftRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void runLiveTuning() {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = kP.get();
        slot0Configs.kI = kI.get();
        slot0Configs.kD = kD.get();
        leftPivotMotor.getConfigurator().apply(slot0Configs);
        rightPivotMotor.getConfigurator().apply(slot0Configs);
        // TODO: Don't do this every loop, only when the values change. This is just a temporary solution for now.
    }
    
}
