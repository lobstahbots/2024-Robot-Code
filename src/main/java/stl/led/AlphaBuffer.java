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

    public static AlphaBuffer invert(AlphaBuffer a) {
        double[] result = new double[a.buffer.length];
        for (int i = 0; i < a.buffer.length; i++) {
            result[i] = 1 - a.buffer[i];
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer multiply(int outputLength, AlphaBuffer a, AlphaBuffer b) {
        double[] result = new double[outputLength];
        int contentLength = Math.min(outputLength, Math.min(a.buffer.length, b.buffer.length));
        for (int i = 0; i < contentLength; i++) {
            result[i] = a.buffer[i] * b.buffer[i];
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer layer(int outputLength, AlphaBuffer... layers) {
        double[] result = new double[outputLength];
        for (AlphaBuffer layer : layers) {
            for (int i = 0; i < Math.min(outputLength, layer.buffer.length); i++) {
                result[i] = result[i] + layer.buffer[i] * (1 - result[i]);
            }
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer concat(int outputLength, AlphaBuffer... segments) {
        double[] result = new double[outputLength];
        int i = 0;
        for (AlphaBuffer segment : segments) {
            for (int j = 0; j < segment.buffer.length; j++) {
                if (i >= outputLength) return new AlphaBuffer(result);
                result[i] = segment.buffer[j];
                i++;
            }
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer concat(AlphaBuffer... segments) {
        int length = 0;
        for (AlphaBuffer segment : segments) {
            length += segment.buffer.length;
        }
        return concat(length, segments);
    }

    public static AlphaBuffer crop(int length, AlphaBuffer input) {
        double[] result = new double[length];
        for (int i = 0; i < Math.min(length, input.buffer.length); i++) {
            result[i] = input.buffer[i];
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer flip(AlphaBuffer input) {
        double[] result = new double[input.buffer.length];
        for (int i = 0; i < input.buffer.length; i++) {
            result[i] = input.buffer[input.buffer.length - i - 1];
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer tile(int length, AlphaBuffer source) {
        double[] result = new double[length];
        for (int i = 0; i < length; i++) {
            int j = Math.floorMod(i, source.buffer.length);
            result[i] = source.buffer[j];
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer repeat(int times, AlphaBuffer source) {
        return tile(times * source.buffer.length, source);
    }

    public static AlphaBuffer wrappedShift(int outputLength, int offset, AlphaBuffer input) {
        double[] result = new double[outputLength];
        for (int i = 0; i < outputLength; i++) {
            result[i] = input.buffer[Math.floorMod(i - offset, input.buffer.length)];
        }
        return new AlphaBuffer(result);
    }

    public static AlphaBuffer cycle(int offset, AlphaBuffer input) {
        return wrappedShift(input.buffer.length, offset, input);
    }

    public static AlphaBuffer shift(int outputLength, int offset, AlphaBuffer input) {
        double[] result = new double[outputLength];
        for (int i = Math.max(0, -offset); i < Math.min(input.buffer.length, outputLength - offset); i++) {
            int j = i + offset;
            result[j] = input.buffer[i];
        }
        return new AlphaBuffer(result);
    }
}
