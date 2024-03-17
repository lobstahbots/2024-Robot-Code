package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    LEDIO io;

    public LEDs(LEDIO io) {
        this.io = io;
    }

    public void periodic() {
        
    }
}
