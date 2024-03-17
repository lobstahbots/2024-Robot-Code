package stl.led;

import java.util.Arrays;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class MaskedLEDBuffer {
    public final AddressableLEDBuffer color;
    public final double[] alpha;
    public final int length;

    public MaskedLEDBuffer(AddressableLEDBuffer ledBuffer, double[] alpha) {
        this.color = ledBuffer;
        this.alpha = alpha;
        this.length = alpha.length;
        if (ledBuffer.getLength() != alpha.length) {
            throw new IllegalArgumentException("Length of color and alpha must be the same");
        }
    }

    public MaskedLEDBuffer(AddressableLEDBuffer ledBuffer, double alpha) {
        this(ledBuffer, new double[ledBuffer.getLength()]);
        Arrays.fill(this.alpha, alpha);
    }

    public MaskedLEDBuffer(AddressableLEDBuffer ledBuffer) {
        this(ledBuffer, 1);
    }

    public AddressableLEDBuffer flatten() {
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
}
