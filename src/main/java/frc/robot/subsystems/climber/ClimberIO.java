package frc.robot.subsystems.climber;

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
        public double leftClimberCurrent = 0.0;
        public double rightClimberCurrent = 0.0;
    }

    /* Moves the left side climber. */
    public default void moveLeftClimber() {}

    /* Moves the right side climber. */
    public default void moveRightClimber() {}
    
    /* Stops the climber motors. */
    public default void stopClimber() {}

    public default void updateInputs(ClimberIOInputs inputs) {}
}
