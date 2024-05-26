package stl.led;

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
}
