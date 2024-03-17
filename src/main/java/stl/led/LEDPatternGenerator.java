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
    public static MaskedLEDBuffer solid(int length, Color color, double alpha) {
        MaskedLEDBuffer ledBuffer = new MaskedLEDBuffer(new AddressableLEDBuffer(length), alpha);
        for (int i = 0; i < ledBuffer.length; i++) {
            ledBuffer.color.setLED(i, color);
        }
        return ledBuffer;
    }

    public static MaskedLEDBuffer solid(int length, Color color) {
        return solid(length, color, 1);
    }
    
    public static MaskedLEDBuffer stack(int length, MaskedLEDBuffer... layers) {
        MaskedLEDBuffer output = new MaskedLEDBuffer(length);
        for (MaskedLEDBuffer layer : layers) {
            for (int i = 0; i < Math.min(length, layer.color.getLength()); i++) {
                output.color.setRGB(i,
                    (int) (layer.color.getRed(i) * layer.alpha[i] + output.color.getRed(i) * output.alpha[i] * (1 - layer.alpha[i])),
                    (int) (layer.color.getGreen(i) * layer.alpha[i] + output.color.getGreen(i) * output.alpha[i] * (1 - layer.alpha[i])),
                    (int) (layer.color.getBlue(i) * layer.alpha[i] + output.color.getBlue(i) * output.alpha[i] * (1 - layer.alpha[i]))
                );

                output.alpha[i] = output.alpha[i] + layer.alpha[i] * (1 - output.alpha[i]);
            }
        }
        return output;
    }

    public static MaskedLEDBuffer wrappedTranslate(int length, MaskedLEDBuffer source, int offset) {
        MaskedLEDBuffer translated = new MaskedLEDBuffer(new AddressableLEDBuffer(length), 0);
        for (int i = 0; i < source.length; i++) {
            int j = Math.floorMod(i + offset, length);
            translated.color.setLED(j, source.color.getLED(i));
            translated.alpha[j] = source.alpha[i];
        }
        return translated;
    }

    public static MaskedLEDBuffer translate(int length, MaskedLEDBuffer source, int offset) {
        MaskedLEDBuffer translated = new MaskedLEDBuffer(new AddressableLEDBuffer(length), 0);
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
    
    public MaskedLEDBuffer solid(Color color, double alpha) {
        return solid(length, color, alpha);
    }

    public MaskedLEDBuffer solid(Color color) {
        return solid(length, color);
    }

    public MaskedLEDBuffer stack(MaskedLEDBuffer... layers) {
        return stack(length, layers);
    }
}
