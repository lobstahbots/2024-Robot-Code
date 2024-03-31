package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double indexerMotorVoltage = 0.0;
        public double indexerMotorTemperature = 0.0;
        public double indexerMotorCurrent = 0.0;
        public boolean intakeBeamBroken = false;
        public boolean flywheelBeamBroken = false;
    }
    
    public default void setIndexerMotorSpeed(double indexerMotorSpeed) {}
    
    public default void stopIndexerMotor() {}
    
    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void periodic() {}
}
