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
     * @param upperShooterMotorSpeed The speed to set the upper shooter to.
     * @param lowerShooterMotorSpeed The speed to set the lower shooter to.
     */
    public default void setShooterMotorSpeed(double upperShooterMotorSpeed, double lowerShooterMotorSpeed) {}
    
    public default void stopShooterMotor() {}

    public default void setIdleMode() {}
    
    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void periodic() {}
}
