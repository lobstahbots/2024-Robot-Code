// Adapted from 6328 Mechanical Advantage's 2023 Robot Code

package stl.led;

import java.util.Arrays;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Generates and composites LED patterns
 * 
 * Adapted from 6328 Mechanical Advantage's 2023 Robot Code
 */
public class LobstahLEDBuffer {
    public final AddressableLEDBuffer color;
    public final double[] alpha;
    public final int length;

    public LobstahLEDBuffer(AddressableLEDBuffer ledBuffer, double[] alpha) {
        this.color = ledBuffer;
        this.alpha = alpha;
        this.length = alpha.length;
        if (ledBuffer.getLength() != alpha.length) {
            throw new IllegalArgumentException("Length of color and alpha must be the same");
        }
    }

    public LobstahLEDBuffer(int[] red, int[] green, int[] blue, double[] alpha) {
        this(new AddressableLEDBuffer(red.length), alpha);
        if (red.length != green.length || red.length != blue.length) {
            throw new IllegalArgumentException("Length of red, green, and blue must be the same");
        }
        for (int i = 0; i < length; i++) {
            color.setRGB(i, red[i], green[i], blue[i]);
        }
    }

    public LobstahLEDBuffer(AddressableLEDBuffer ledBuffer, double alpha) {
        this(ledBuffer, new double[ledBuffer.getLength()]);
        Arrays.fill(this.alpha, alpha);
    }

    public LobstahLEDBuffer(AddressableLEDBuffer ledBuffer) {
        this(ledBuffer, 1);
    }

    public LobstahLEDBuffer(int length, double alpha) {
        this(new AddressableLEDBuffer(length), alpha);
    }

    public LobstahLEDBuffer(int length) {
        this(new AddressableLEDBuffer(length), new double[length]);
    }

    public AddressableLEDBuffer toAdressableLEDBuffer() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(color.getLength());
        for (int i = 0; i < color.getLength(); i++) {
            buffer.setRGB(i,
                (int) (color.getRed(i) * alpha[i]),
                (int) (color.getGreen(i) * alpha[i]),
                (int) (color.getBlue(i) * alpha[i])
            );
        }
        return buffer;
    }

    public static LobstahLEDBuffer solid(int length, Color color, double alpha) {
        LobstahLEDBuffer ledBuffer = new LobstahLEDBuffer(length, alpha);
        for (int i = 0; i < ledBuffer.length; i++) {
            ledBuffer.color.setLED(i, color);
        }
        return ledBuffer;
    }

    public static LobstahLEDBuffer solid(int length, Color color) {
        return solid(length, color, 1);
    }
    
    public static LobstahLEDBuffer layer(int outputLength, LobstahLEDBuffer... layers) {
        LobstahLEDBuffer output = new LobstahLEDBuffer(outputLength);
        for (LobstahLEDBuffer layer : layers) {
            for (int i = 0; i < Math.min(outputLength, layer.color.getLength()); i++) {
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

    public static LobstahLEDBuffer concat(int outputLength, LobstahLEDBuffer... layers) { // TODO: Implement concat
        LobstahLEDBuffer output = new LobstahLEDBuffer(outputLength);
        int i = 0;
        for (LobstahLEDBuffer layer : layers) {
            for (int j = 0; j < layer.length; j++) {
                if (i >= outputLength) return output;
                output.color.setLED(i, layer.color.getLED(j));
                output.alpha[i] = layer.alpha[j];
                i++;
            }
        }
        return output;
    }

    public static LobstahLEDBuffer concat(LobstahLEDBuffer... layers) {
        int length = 0;
        for (LobstahLEDBuffer layer : layers) {
            length += layer.length;
        }
        return concat(length, layers);
    }

    public static LobstahLEDBuffer crop(int length, LobstahLEDBuffer input) {
        LobstahLEDBuffer cropped = new LobstahLEDBuffer(length);
        for (int i = 0; i < Math.min(length, input.length); i++) {
            cropped.color.setLED(i, input.color.getLED(i));
            cropped.alpha[i] = input.alpha[i];
        }
        return cropped;
    }

    public static LobstahLEDBuffer flip(LobstahLEDBuffer input) {
        LobstahLEDBuffer flipped = new LobstahLEDBuffer(input.length);
        for (int i = 0; i < input.length; i++) {
            flipped.color.setLED(i, input.color.getLED(input.length - i - 1));
            flipped.alpha[i] = input.alpha[input.length - i - 1];
        }
        return flipped;
    }

    public static LobstahLEDBuffer tile(int length, LobstahLEDBuffer source) {
        LobstahLEDBuffer tiled = new LobstahLEDBuffer(length);
        for (int i = 0; i < length; i++) {
            int j = Math.floorMod(i, source.length);
            tiled.color.setLED(i, source.color.getLED(j));
            tiled.alpha[i] = source.alpha[j];
        }
        return tiled;
    }

    public static LobstahLEDBuffer repeat(int times, LobstahLEDBuffer source) {
        return tile(times * source.length, source);
    }

    public static LobstahLEDBuffer wrappedShift(int outputLength, int offset, LobstahLEDBuffer input) {
        LobstahLEDBuffer translated = new LobstahLEDBuffer(outputLength);
        for (int i = 0; i < input.length; i++) {
            int j = Math.floorMod(i + offset, outputLength);
            translated.color.setLED(j, input.color.getLED(i));
            translated.alpha[j] = input.alpha[i];
        }
        return translated;
    }

    public static LobstahLEDBuffer cycle(int offset, LobstahLEDBuffer input) {
        return wrappedShift(input.length, offset, input);
    }

    public static LobstahLEDBuffer shift(int outputLength, int offset, LobstahLEDBuffer input) {
        LobstahLEDBuffer translated = new LobstahLEDBuffer(outputLength);
        for (int i = Math.max(0, -offset); i < Math.min(input.length, outputLength - offset); i++) {
            int j = i + offset;
            translated.color.setLED(j, input.color.getLED(i));
            translated.alpha[j] = input.alpha[i];
        }
        return translated;
    }
}
