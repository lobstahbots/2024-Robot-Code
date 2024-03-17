// Adapted from 6328 Mechanical Advantage's 2023 Robot Code

package stl.led;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Generates and composites LED patterns
 * 
 * Adapted from 6328 Mechanical Advantage's 2023 Robot Code
 */
public class LEDPatternGenerator {
    public static AddressableLEDBuffer solid(int length, Color color) {
        AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(length);
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
        return ledBuffer;
    }
    
    public static MaskedLEDBuffer stack(int length, MaskedLEDBuffer... layers) {
        MaskedLEDBuffer buffer = new MaskedLEDBuffer(new AddressableLEDBuffer(length));
        for (MaskedLEDBuffer layer : layers) {
            for (int i = 0; i < Math.min(length, layer.color.getLength()); i++) {
                buffer.color.setRGB(i,
                    (int) (layer.color.getRed(i) + (1 - layer.alpha[i]) * buffer.color.getRed(i)),
                    (int) (layer.color.getGreen(i) + (1 - layer.alpha[i]) * buffer.color.getGreen(i)),
                    (int) (layer.color.getBlue(i) + (1 - layer.alpha[i]) * buffer.color.getBlue(i))
                );

                buffer.alpha[i] = buffer.alpha[i] + layer.alpha[i] * (1 - buffer.alpha[i]);
            }
        }
        return buffer;
    }

    public static MaskedLEDBuffer wrappedTranslate(int length, MaskedLEDBuffer source, int offset) {
        MaskedLEDBuffer translated = new MaskedLEDBuffer(new AddressableLEDBuffer(length));
        for (int i = 0; i < source.length; i++) {
            int j = Math.floorMod(i + offset, length);
            translated.color.setLED(j, source.color.getLED(i));
            translated.alpha[j] = source.alpha[i];
        }
        return translated;
    }

    public static MaskedLEDBuffer translate(int length, MaskedLEDBuffer source, int offset) {
        MaskedLEDBuffer translated = new MaskedLEDBuffer(new AddressableLEDBuffer(length));
        for (int i = Math.max(0, -offset); i < Math.min(source.length, length - offset); i++) {
            int j = i + offset;
            translated.color.setLED(j, source.color.getLED(i));
            translated.alpha[j] = source.alpha[i];
        }
        return translated;
    }
    
    int length;
    
    LEDPatternGenerator(int length) {
        this.length = length;
    }
    
    public AddressableLEDBuffer solid(Color color) {
        return solid(length, color);
    }

    public MaskedLEDBuffer stack(MaskedLEDBuffer... layers) {
        return stack(length, layers);
    }
}
