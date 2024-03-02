package frc.robot.subsystems.intake;

import java.util.Arrays;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import stl.tempControl.MonitoredSparkMax;
import stl.tempControl.TemperatureMonitor;

public class IntakeSparkMax implements IntakeIO {
    private final MonitoredSparkMax intakeMotor;
    private final MonitoredSparkMax indexerMotor;
    private final TemperatureMonitor monitor;
    /**
     * Creates a new IntakeSparkMax
     * @param intakeMotorID
     */
    public IntakeSparkMax(int intakeMotorID, int indexerMotorID){
        this.intakeMotor = new MonitoredSparkMax(intakeMotorID, MotorType.kBrushless, "Intake motor");
        this.indexerMotor = new MonitoredSparkMax(indexerMotorID, MotorType.kBrushless, "Indexer motor");
        this.intakeMotor.setIdleMode(IdleMode.kBrake);
        this.intakeMotor.setInverted(true);
        this.intakeMotor.setSmartCurrentLimit(40);

        monitor = new TemperatureMonitor(Arrays.asList(intakeMotor, indexerMotor));
    }
    /**
     * Sets the intake motor speed to the given speed.
     * @param intakeMotorSpeed The speed to set the motor to
     */
    public void setIntakeMotorSpeed(double intakeMotorSpeed){
        intakeMotor.set(intakeMotorSpeed);
    }

    /**
     * Sets the indexer motor speed to the given speed.
     * @param indexerMotorSpeed The speed to set the motor to
     */
    public void setIndexerMotorSpeed(double indexerMotorSpeed){
        indexerMotor.set(indexerMotorSpeed);
    }

    /**
     * Stops the intake motor.
     */
    public void stopIntakeMotor(){
        intakeMotor.stopMotor();
    }

    /**
     * Stops the indexer motor.
     */
    public void stopIndexerMotor(){
        indexerMotor.stopMotor();
    }
  
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorVoltage = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
        inputs.intakeMotorTemperature = intakeMotor.getMotorTemperature();
        inputs.intakeMotorCurrent = intakeMotor.getOutputCurrent();
        inputs.indexerMotorVoltage = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
        inputs.indexerMotorTemperature = indexerMotor.getMotorTemperature();
        inputs.indexerMotorCurrent = indexerMotor.getOutputCurrent();
    }

    public void periodic() {
        monitor.monitor();
    }
}
