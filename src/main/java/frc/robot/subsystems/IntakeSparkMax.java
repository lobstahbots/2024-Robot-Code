package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class IntakeSparkMax implements IntakeIO{
    private final CANSparkMax intakeMotor;
    
    public IntakeSparkMax(int intakeMotorID){
        this.intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
        this.intakeMotor.setIdleMode(IdleMode.kBrake);
        this.intakeMotor.setInverted(false);
        this.intakeMotor.setSmartCurrentLimit(40);
    }
    //@param motor speed
    // set the intake motor speed
    public void setIntakeMotorSpeed(double intakeMotorSpeed){
        intakeMotor.set(intakeMotorSpeed);
    }
    // stops intake motor
    public void stopIntakeMotor(){
        intakeMotor.stopMotor();
    }
    // update intake inputs
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorVoltage = intakeMotor.getBusVoltage() * intakeMotor.getAppliedOutput();
        inputs.intakeMotorTemperature = intakeMotor.getMotorTemperature();
        inputs.intakeMotorCurrent = intakeMotor.getOutputCurrent();
    }
}
