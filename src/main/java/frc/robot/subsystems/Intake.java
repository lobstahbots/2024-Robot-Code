package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase{
    private final CANSparkMax intakeMotor;

    public Intake(int intakeMotorID){
        this.intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
        this.intakeMotor.setIdleMode(IdleMode.kBrake);
        this.intakeMotor.setInverted(false);
        this.intakeMotor.setSmartCurrentLimit(40);
    }

    public void setIMotorSpeed(double intakeSpeed){
        intakeMotor.set(intakeSpeed);
    }

    public void stopMotor(){
        intakeMotor.stopMotor();
    }
}
