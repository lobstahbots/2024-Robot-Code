// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package stl.tempControl;

import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TempConstants;
import stl.networkalerts.Alert;
import stl.networkalerts.Alert.AlertType;

/**
 * Class to monitor the temperature of some items.
 */
public class TemperatureMonitor {
    public static interface Monitorable {
        /**
         * Sets the disabled state of the monitorable.
         * @param disable Whether or not to disable.
         */
        public void setDisabled(boolean disable);
        /**
         * Gets the disabled state of the monitorable.
         * @return Whether or not it is disabled.
         */
        public boolean getDisabled();
        /**
         * Gets the motor temperature.
         * @return The temperature.
         */
        public double getMotorTemperature();
        /**
         * Gets the label for this.
         * @return The label.
         */
        public String getLabel();
    }
    private double overheatTime = -1;
    private boolean disabled = false;
    private List<Monitorable> motors;
    private final HashMap<String, Alert> alerts;

    public TemperatureMonitor(List<Monitorable> motors) {
        this.motors = motors;
        alerts = new HashMap<>(motors.size(), 1); // Load factor can be 1 because the size will never change
        for (var motor: motors) {
            alerts.put(motor.getLabel(), new Alert(
                String.format(
                    "%s has overheated above %d C. It and related motors will be disabled until its temperature drops below %d C.",
                    motor.getLabel(),
                    TempConstants.OVERHEAT_TEMP,
                    TempConstants.SAFE_TEMP
                ),
                AlertType.ERROR
            ));
        }
    }

    public void monitor() {
        boolean safe = true;
        for (Monitorable motor : motors) {
            double temp = motor.getMotorTemperature();
            if (temp > TempConstants.SAFE_TEMP) safe = false;
            else alerts.get(motor.getLabel()).set(false);
            if (temp > TempConstants.OVERHEAT_TEMP && overheatTime == -1) overheatTime = Timer.getFPGATimestamp();
            else if (temp > TempConstants.OVERHEAT_TEMP && Timer.getFPGATimestamp() - overheatTime >= 2) {
                disabled = true;
                alerts.get(motor.getLabel()).set(true);
            }
            if (disabled) {
                motor.setDisabled(true);
            }
        }
        if (safe) {
            disabled = false;
            overheatTime = -1;
        }
    }
}
