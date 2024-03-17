package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import stl.led.LEDPatternGenerator;
import stl.led.MaskedLEDBuffer;

public class LEDs extends SubsystemBase {
    LEDIO io;
    

    public LEDs(LEDIO io) {
        this.io = io;
    }

    public void periodic() {
        // AddressableLEDBuffer red = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);
        // for (int i = 0; i < red.getLength(); i++) {
        //     red.setRGB(i, 255, 50, 60);
        // }

        int offset = (int)(Timer.getFPGATimestamp() * 4) - 20;

        MaskedLEDBuffer lala = LEDPatternGenerator.translate(LEDConstants.LED_LENGTH, new MaskedLEDBuffer(LEDPatternGenerator.solid(60, new Color(255, 255, 255))), offset);

        io.setData(lala.flatten());
    }
}
