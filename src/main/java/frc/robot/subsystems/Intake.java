package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Intake extends SubsystemBase{
    private final CANSparkMax iMotor;
//Create new intake *wow*
    public Intake(int iMotorID){
        //look! it's a neo or smthng!
        this.iMotor = new CANSparkMax(iMotorID, MotorType.kBrushless);
        this.iMotor.setIdleMode(IdleMode.kBrake);
        this.iMotor.setInverted(false);
        this.iMotor.setSmartCurrentLimit(25);
    }

//we have a Intake.setIMotorSpeed to... set the intake speed!
    public void setIMotorSpeed(double intakeSpeed){
        iMotor.set(intakeSpeed);


    }
//gee I wonder what this could do
    public void stopMotor(){
        iMotor.stopMotor();

    }
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

}
