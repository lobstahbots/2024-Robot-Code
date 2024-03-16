package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
    LEDIO io;
    

    public LEDs(LEDIO io) {
        this.io = io;
    }

    public void periodic() {
        AddressableLEDBuffer red = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);
        for (int i = 0; i < red.getLength(); i++) {
            red.setRGB(i, 255, 50, 60);
        }

        io.setData(red);
    }
}
