package stl.led;

import java.awt.image.BufferedImage;
import java.awt.image.WritableRaster;
import java.io.File;

import javax.imageio.ImageIO;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class LEDDevTools {
    public static Color tunableColor(String key, Color defaultColor) {
        String smartDashboardValue = SmartDashboard.getString("leds/color/"+key, "");
        if (smartDashboardValue == "") {
            SmartDashboard.putString("leds/color/"+key, defaultColor.toHexString());
            return defaultColor;
        } else {
            try {
                return new Color(smartDashboardValue);
            } catch (Exception e) {
                return defaultColor;
            }
        }
    }

    public static void logBitmap(AddressableLEDBuffer buffer, File file) {
        BufferedImage image = new BufferedImage(buffer.getLength(), 1, BufferedImage.TYPE_INT_RGB);
        WritableRaster raster = image.getRaster();

        for (int i = 0; i < buffer.getLength(); i++) {
            raster.setPixel(i, 0, new int[] {buffer.getRed(i), buffer.getGreen(i), buffer.getBlue(i)});
        }

        try {
            ImageIO.write(image, "bmp", file);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static int logIndex = 0;
    public static void logBitmap(AddressableLEDBuffer buffer) {
        logBitmap(buffer, new File(Filesystem.getOperatingDirectory(), "led_log/"+logIndex+".bmp"));
        logIndex++;
    }
}
