package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LEDConstants.LengthConstants;
import stl.led.LobstahLEDBuffer;

import static stl.led.LEDDevTools.*;

public class LEDsDemo extends LEDs {
    public LEDsDemo(LEDIO io) {
        super(io);
        SmartDashboard.putBoolean("ledCapture", false);
    }

    public void periodic() {
        loadingNotifier.stop();

        AddressableLEDBuffer buffer = LobstahLEDBuffer
                .layer(LengthConstants.TOTAL, LobstahLEDBuffer.solid(LengthConstants.TOTAL, Color.kWhite))
                .toAdressableLEDBuffer();

        io.setData(buffer);
      
        if (SmartDashboard.getBoolean("ledCapture", false)) {
            logBitmap(buffer);
        }
    }
}
