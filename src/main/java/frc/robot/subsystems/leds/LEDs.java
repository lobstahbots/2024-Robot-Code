// Adapted from 6328 Mechanical Advantage's 2023 Robot Code

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import stl.led.AlphaBuffer;
import stl.led.AnimationEasing;
import stl.led.LobstahLEDBuffer;

public class LEDs extends SubsystemBase {
    LEDIO io;

    boolean possession = false;

    public LEDs(LEDIO io) {
        this.io = io;
        
        loadingNotifier.startPeriodic(0.02);
    }

    public void setPossession(boolean value) {
        possession = value;
        if (possession) possessionSignalTimer.restart();
    }

    public void periodic() {
        loadingNotifier.stop();

        io.setData(LobstahLEDBuffer.layer(LEDConstants.LED_LENGTH,
                posessionSignal(),
                posessionIndicator(),
                inMatchPattern()).toAdressableLEDBuffer());
    }

    private final Notifier loadingNotifier = new Notifier(() -> {
        synchronized (this) {
            System.out.println("Loading");
            io.setData(LobstahLEDBuffer.solid(3, Color.kWhite)
                    .opacity(AnimationEasing.sine(System.currentTimeMillis(), 1000, 0)).toAdressableLEDBuffer());
        }
    });
    
    Timer possessionSignalTimer = new Timer();
    LobstahLEDBuffer posessionSignal() {
        double t = possessionSignalTimer.get();
        if (t>= 1 || !possession) return null;

        return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(77, 255, 79))
            .opacity(1-t);
    }

    LobstahLEDBuffer posessionIndicator() {
        if (!possession) return null;
        return LobstahLEDBuffer.solid(5, new Color(77, 255, 79))
            .wrappedShift(LEDConstants.LED_LENGTH, -5);
    }

    LobstahLEDBuffer inMatchPattern() {
        return LobstahLEDBuffer.solid(LEDConstants.LED_LENGTH, new Color(255, 69, 118))
                .mask(AlphaBuffer.sine(LEDConstants.LED_LENGTH, 10, Timer.getFPGATimestamp() * 5));
    }
}
