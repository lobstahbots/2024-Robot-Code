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
    public final AlphaBuffer alpha;
    public final int length;

    public LobstahLEDBuffer(AddressableLEDBuffer ledBuffer, AlphaBuffer alpha) {
        this.color = ledBuffer;
        this.alpha = alpha;
        this.length = alpha.buffer.length;
        if (ledBuffer.getLength() != alpha.buffer.length) {
            throw new IllegalArgumentException("Length of color and alpha must be the same");
        }
    }

    public LobstahLEDBuffer(int[] red, int[] green, int[] blue, double[] alpha) {
        this(new AddressableLEDBuffer(red.length), new AlphaBuffer(alpha));
        if (red.length != green.length || red.length != blue.length) {
            throw new IllegalArgumentException("Length of red, green, and blue must be the same");
        }
        for (int i = 0; i < length; i++) {
            color.setRGB(i, red[i], green[i], blue[i]);
        }
    }

    public LobstahLEDBuffer(AddressableLEDBuffer ledBuffer, double alpha) {
        this(ledBuffer, new AlphaBuffer(ledBuffer.getLength(), alpha));
    }

    public LobstahLEDBuffer(AddressableLEDBuffer ledBuffer) {
        this(ledBuffer, 1);
    }

    protected LobstahLEDBuffer(int length, double alpha) {
        this(new AddressableLEDBuffer(length), alpha);
    }

    public LobstahLEDBuffer(int length) {
        this(new AddressableLEDBuffer(length), new AlphaBuffer(length));
    }

    public AddressableLEDBuffer toAdressableLEDBuffer() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(color.getLength());
        for (int i = 0; i < color.getLength(); i++) {
            buffer.setRGB(i,
                (int) (color.getRed(i) * alpha.buffer[i]),
                (int) (color.getGreen(i) * alpha.buffer[i]),
                (int) (color.getBlue(i) * alpha.buffer[i])
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
                    (int) (layer.color.getRed(i) * layer.alpha.buffer[i] + output.color.getRed(i) * output.alpha.buffer[i] * (1 - layer.alpha.buffer[i])),
                    (int) (layer.color.getGreen(i) * layer.alpha.buffer[i] + output.color.getGreen(i) * output.alpha.buffer[i] * (1 - layer.alpha.buffer[i])),
                    (int) (layer.color.getBlue(i) * layer.alpha.buffer[i] + output.color.getBlue(i) * output.alpha.buffer[i] * (1 - layer.alpha.buffer[i]))
                );

                output.alpha.buffer[i] = output.alpha.buffer[i] + layer.alpha.buffer[i] * (1 - output.alpha.buffer[i]);
            }
        }
        return output;
    }

    public static LobstahLEDBuffer concat(int outputLength, LobstahLEDBuffer... segments) {
        LobstahLEDBuffer output = new LobstahLEDBuffer(outputLength);
        int i = 0;
        for (LobstahLEDBuffer segment : segments) {
            for (int j = 0; j < segment.length; j++) {
                if (i >= outputLength) return output;
                output.color.setLED(i, segment.color.getLED(j));
                output.alpha.buffer[i] = segment.alpha.buffer[j];
                i++;
            }
        }
        return output;
    }

    public static LobstahLEDBuffer concat(LobstahLEDBuffer... segments) {
        int length = 0;
        for (LobstahLEDBuffer segment : segments) {
            length += segment.length;
        }
        return concat(length, segments);
    }

    public static LobstahLEDBuffer crop(int length, LobstahLEDBuffer input) {
        LobstahLEDBuffer cropped = new LobstahLEDBuffer(length);
        for (int i = 0; i < Math.min(length, input.length); i++) {
            cropped.color.setLED(i, input.color.getLED(i));
            cropped.alpha.buffer[i] = input.alpha.buffer[i];
        }
        return cropped;
    }

    public static LobstahLEDBuffer flip(LobstahLEDBuffer input) {
        LobstahLEDBuffer flipped = new LobstahLEDBuffer(input.length);
        for (int i = 0; i < input.length; i++) {
            flipped.color.setLED(i, input.color.getLED(input.length - i - 1));
            flipped.alpha.buffer[i] = input.alpha.buffer[input.length - i - 1];
        }
        return flipped;
    }

    public static LobstahLEDBuffer tile(int length, LobstahLEDBuffer source) {
        LobstahLEDBuffer tiled = new LobstahLEDBuffer(length);
        for (int i = 0; i < length; i++) {
            int j = Math.floorMod(i, source.length);
            tiled.color.setLED(i, source.color.getLED(j));
            tiled.alpha.buffer[i] = source.alpha.buffer[j];
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
            translated.alpha.buffer[j] = input.alpha.buffer[i];
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
            translated.alpha.buffer[j] = input.alpha.buffer[i];
        }
        return translated;
    }

    public LobstahLEDBuffer layerAbove(LobstahLEDBuffer background) {
        return layer(length, background, this);
    }

    public LobstahLEDBuffer layerBelow(LobstahLEDBuffer foreground) {
        return layer(length, this, foreground);
    }

    public LobstahLEDBuffer append(LobstahLEDBuffer other) {
        return concat(this, other);
    }

    public LobstahLEDBuffer prepend(LobstahLEDBuffer other) {
        return concat(other, this);
    }

    public LobstahLEDBuffer crop(int length) {
        return crop(length, this);
    }

    public LobstahLEDBuffer flip() {
        return flip(this);
    }

    public LobstahLEDBuffer tile(int length) {
        return tile(length, this);
    }

    public LobstahLEDBuffer repeat(int times) {
        return repeat(times, this);
    }

    public LobstahLEDBuffer cycle(int offset) {
        return cycle(offset, this);
    }

    public LobstahLEDBuffer shift(int outputLength, int offset) {
        return shift(outputLength, offset, this);
    }
}
