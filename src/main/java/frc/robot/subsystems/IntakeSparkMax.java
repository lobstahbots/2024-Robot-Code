package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSparkMax implements IntakeIO {
    private final MonitoredSparkMax intakeMotor;
    private final TemperatureMonitor monitor;
    /**
     * Creates a new IntakeSparkMax
     * @param intakeMotorID
     */
    public IntakeSparkMax(int intakeMotorID){
        this.intakeMotor = new MonitoredSparkMax(intakeMotorID, MotorType.kBrushless);
        this.intakeMotor.setIdleMode(IdleMode.kBrake);
        this.intakeMotor.setInverted(false);
        this.intakeMotor.setSmartCurrentLimit(40);

        monitor = new TemperatureMonitor(Arrays.asList(intakeMotor));
    }
    /**
     * Sets the intake motor speed to the given speed.
     * @param intakeMotorSpeed
     */
    public void setIntakeMotorSpeed(double intakeMotorSpeed){
        intakeMotor.set(intakeMotorSpeed);
    }
    /**
     * Stops the intake motor.
     */
    public void stopIntakeMotor(){
        intakeMotor.stopMotor();
    }
  
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorVoltage = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
        inputs.intakeMotorTemperature = intakeMotor.getMotorTemperature();
        inputs.intakeMotorCurrent = intakeMotor.getOutputCurrent();
    }

    public void periodic() {
        monitor.monitor();
    }
}
