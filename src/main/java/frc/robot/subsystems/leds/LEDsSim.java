package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;

public class LEDsSim implements LEDIO {
    AddressableLEDSim leds;

    LEDsSim(int length) {
        leds = new AddressableLEDSim();
        leds.setLength(length);
    }

    public void setData(AddressableLEDBuffer buffer) {
        leds.setData(buffer.m_buffer);
    }
}
