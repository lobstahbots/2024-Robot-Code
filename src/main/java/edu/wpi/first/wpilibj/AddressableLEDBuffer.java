// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Modified to make underlying buffer public/accessible to Simulated LEDs.

package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Buffer storage for Addressable LEDs.
 * 
 * Modified to make underlying buffer public/accessible to Simulated LEDs.
 */
public class AddressableLEDBuffer {
  public byte[] m_buffer;

  /**
   * Constructs a new LED buffer with the specified length.
   *
   * @param length The length of the buffer in pixels
   */
  public AddressableLEDBuffer(int length) {
    m_buffer = new byte[length * 4];
  }

  /**
   * Sets a specific led in the buffer.
   *
   * @param index the index to write
   * @param r the r value [0-255]
   * @param g the g value [0-255]
   * @param b the b value [0-255]
   */
  public void setRGB(int index, int r, int g, int b) {
    m_buffer[index * 4] = (byte) b;
    m_buffer[(index * 4) + 1] = (byte) g;
    m_buffer[(index * 4) + 2] = (byte) r;
    m_buffer[(index * 4) + 3] = 0;
  }

  /**
   * Sets a specific led in the buffer.
   *
   * @param index the index to write
   * @param h the h value [0-180)
   * @param s the s value [0-255]
   * @param v the v value [0-255]
   */
  public void setHSV(final int index, final int h, final int s, final int v) {
    if (s == 0) {
      setRGB(index, v, v, v);
      return;
    }

    // The below algorithm is copied from Color.fromHSV and moved here for
    // performance reasons.

    // Loosely based on
    // https://en.wikipedia.org/wiki/HSL_and_HSV#HSV_to_RGB
    // The hue range is split into 60 degree regions where in each region there
    // is one rgb component at a low value (m), one at a high value (v) and one
    // that changes (X) from low to high (X+m) or high to low (v-X)

    // Difference between highest and lowest value of any rgb component
    final int chroma = (s * v) / 255;

    // Because hue is 0-180 rather than 0-360 use 30 not 60
    final int region = (h / 30) % 6;

    // Remainder converted from 0-30 to 0-255
    final int remainder = (int) Math.round((h % 30) * (255 / 30.0));

    // Value of the lowest rgb component
    final int m = v - chroma;

    // Goes from 0 to chroma as hue increases
    final int X = (chroma * remainder) >> 8;

    switch (region) {
      case 0:
        setRGB(index, v, X + m, m);
        break;
      case 1:
        setRGB(index, v - X, v, m);
        break;
      case 2:
        setRGB(index, m, v, X + m);
        break;
      case 3:
        setRGB(index, m, v - X, v);
        break;
      case 4:
        setRGB(index, X + m, m, v);
        break;
      default:
        setRGB(index, v, m, v - X);
        break;
    }
  }

  /**
   * Sets a specific LED in the buffer.
   *
   * @param index The index to write
   * @param color The color of the LED
   */
  public void setLED(int index, Color color) {
    setRGB(index, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  /**
   * Sets a specific LED in the buffer.
   *
   * @param index The index to write
   * @param color The color of the LED
   */
  public void setLED(int index, Color8Bit color) {
    setRGB(index, color.red, color.green, color.blue);
  }

  /**
   * Gets the buffer length.
   *
   * @return the buffer length
   */
  public int getLength() {
    return m_buffer.length / 4;
  }

  /**
   * Gets the color at the specified index.
   *
   * @param index the index to get
   * @return the LED color at the specified index
   */
  public Color8Bit getLED8Bit(int index) {
    return new Color8Bit(getRed(index), getGreen(index), getBlue(index));
  }

  /**
   * Gets the color at the specified index.
   *
   * @param index the index to get
   * @return the LED color at the specified index
   */
  public Color getLED(int index) {
    return new Color(getRed(index) / 255.0, getGreen(index) / 255.0, getBlue(index) / 255.0);
  }

  /**
   * Gets the red channel of the color at the specified index.
   *
   * @param index the index of the LED to read
   * @return the value of the red channel, from [0, 255]
   */
  public int getRed(int index) {
    return m_buffer[index * 4 + 2] & 0xFF;
  }

  /**
   * Gets the green channel of the color at the specified index.
   *
   * @param index the index of the LED to read
   * @return the value of the green channel, from [0, 255]
   */
  public int getGreen(int index) {
    return m_buffer[index * 4 + 1] & 0xFF;
  }

  /**
   * Gets the blue channel of the color at the specified index.
   *
   * @param index the index of the LED to read
   * @return the value of the blue channel, from [0, 255]
   */
  public int getBlue(int index) {
    return m_buffer[index * 4] & 0xFF;
  }

  /**
   * A functional interface that allows for iteration over an LED buffer without manually writing an
   * indexed for-loop.
   */
  @FunctionalInterface
  public interface IndexedColorIterator {
    /**
     * Accepts an index of an LED in the buffer and the red, green, and blue components of the
     * currently stored color for that LED.
     *
     * @param index the index of the LED in the buffer that the red, green, and blue channels
     *     corresponds to
     * @param r the value of the red channel of the color currently in the buffer at index {@code i}
     * @param g the value of the green channel of the color currently in the buffer at index {@code
     *     i}
     * @param b the value of the blue channel of the color currently in the buffer at index {@code
     *     i}
     */
    void accept(int index, int r, int g, int b);
  }

  /**
   * Iterates over the LEDs in the buffer, starting from index 0. The iterator function is passed
   * the current index of iteration, along with the values for the red, green, and blue components
   * of the color written to the LED at that index.
   *
   * @param iterator the iterator function to call for each LED in the buffer.
   */
  public void forEach(IndexedColorIterator iterator) {
    for (int i = 0; i < getLength(); i++) {
      iterator.accept(i, getRed(i), getGreen(i), getBlue(i));
    }
  }
}
