package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LEDConstants;

public class LEDsReal implements LEDIO {
    AddressableLED leds;

    LEDsReal(int port, int length) {
        leds = new AddressableLED(LEDConstants.LED_PORT);
        leds.setLength(LEDConstants.LED_LENGTH);
        leds.start();
    }

    public void setData(AddressableLEDBuffer buffer) {
        leds.setData(buffer);
    }
}
