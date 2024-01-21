package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public final double leftClimberVelocity = 0.0;
        public final double rightClimberVelocity = 0.0;
        public final double leftClimberPosition = 0.0;
        public final double rightClimberPosition = 0.0;
        public final double leftClimberTemperature = 0.0;
        public final double rightClimberTemperature = 0.0;
        public final double leftClimberVoltage = 0.0;
        public final double rightClimberVoltage = 0.0;
    }

    public default void moveLeftClimber() {}

    public default void moveRightClimber() {}
    
    public default void stopClimber() {}

    public default void getInputs(ClimberIOInputs inputs) {}


}
