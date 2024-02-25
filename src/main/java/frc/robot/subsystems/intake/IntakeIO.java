package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double intakeMotorVoltage = 0.0;
        public double intakeMotorTemperature = 0.0;
        public double intakeMotorCurrent = 0.0;
        public double indexerMotorVoltage = 0.0;
        public double indexerMotorTemperature = 0.0;
        public double indexerMotorCurrent = 0.0;
    }
    /** Sets the intake motor speed.  
     * 
     * @param intakeMotorSpeed The speed to set the intake to.
     */
    public default void setIntakeMotorSpeed(double intakeMotorSpeed) {}

    /** Sets the intake motor speed.  
     * 
     * @param indexerMotorSpeed The speed to set the intake to.
     */
    public default void setIndexerMotorSpeed(double indexerMotorSpeed) {}
    
    /** Stops the intake motor. */
    public default void stopIntakeMotor() {}

    /** Stops the indexer motor. */
    public default void stopIndexerMotor() {}
    
    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void periodic() {}
}
