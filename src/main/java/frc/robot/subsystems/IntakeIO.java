package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double intakeMotorVoltage = 0.0;
        public double intakeMotorTemperature = 0.0;
        public double intakeMotorCurrent = 0.0;
    }
    
    public default void setIntakeMotorSpeed(double intakeMotorSpeed) {}
    
    public default void stopIntakeMotor() {}
    
    public default void updateInputs(IntakeIOInputs inputs) {}
}
