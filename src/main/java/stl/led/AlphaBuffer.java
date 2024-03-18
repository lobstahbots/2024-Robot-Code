package stl.led;

import java.util.Arrays;

public class AlphaBuffer {
    // A wrapper for an array of doubles
    public final double[] buffer;

    public AlphaBuffer(double[] alpha) {
        this.buffer = alpha;
    }

    public AlphaBuffer(int length) {
        this(new double[length]);
    }

    public AlphaBuffer(int length, double alpha) {
        this(length);
        Arrays.fill(buffer, alpha);
    }
}
