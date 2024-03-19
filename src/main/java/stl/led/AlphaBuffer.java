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

    public static AlphaBuffer multiply(AlphaBuffer a, AlphaBuffer b) {
        double[] result = new double[a.buffer.length];
        for (int i = 0; i < a.buffer.length; i++) {
            result[i] = a.buffer[i] * b.buffer[i];
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer layer(AlphaBuffer a, AlphaBuffer b) {
        double[] result = new double[a.buffer.length];
        for (int i = 0; i < a.buffer.length; i++) {
            result[i] = a.buffer[i] + b.buffer[i] * (1 - a.buffer[i]);
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer invert(AlphaBuffer a) {
        double[] result = new double[a.buffer.length];
        for (int i = 0; i < a.buffer.length; i++) {
            result[i] = 1 - a.buffer[i];
        }
        return new AlphaBuffer(result);
    }
}
