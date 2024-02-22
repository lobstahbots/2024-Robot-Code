package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public double upperShooterMotorVoltage = 0.0;
        public double upperShooterMotorTemperature = 0.0;
        public double upperShooterMotorCurrent = 0.0;
        public double upperShooterMotorVelocity = 0.0;
        public double lowerShooterMotorVoltage = 0.0;
        public double lowerShooterMotorTemperature = 0.0;
        public double lowerShooterMotorCurrent = 0.0;
        public double lowerShooterMotorVelocity = 0.0;
    }
    /** Sets the shooter motor speed.  
     * 
     * @param shooterMotorSpeed The speed to set the shooter to.
     */
    public default void setUpperShooterMotorSpeed(double shooterMotorSpeed) {}

    public default void setLowerShooterMotorSpeed(double shooterMotorSpeed) {}
    
    public default void stopShooterMotor() {}
    
    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void periodic() {}
}
