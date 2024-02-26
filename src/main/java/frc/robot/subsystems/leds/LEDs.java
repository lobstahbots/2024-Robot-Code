package frc.robot.subsystems.leds;

public class LEDs {
    LEDIO io;
    

    public LEDs(LEDIO io) {
        this.io = io;
    }

    public void periodic() {
        io.setData(null);
    }
}
