package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double leftClimberVelocity = 0.0;
        public double rightClimberVelocity = 0.0;
        public double leftClimberPosition = 0.0;
        public double rightClimberPosition = 0.0;
        public double leftClimberTemperature = 0.0;
        public double rightClimberTemperature = 0.0;
        public double leftClimberVoltage = 0.0;
        public double rightClimberVoltage = 0.0;
    }

    public default void moveLeftClimber() {}

    public default void moveRightClimber() {}
    
    public default void stopClimber() {}

    public default void updateInputs(ClimberIOInputs inputs) {}
}
