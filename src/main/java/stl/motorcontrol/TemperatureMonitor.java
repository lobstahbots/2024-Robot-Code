// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package stl.motorcontrol;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TempConstants;

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
    }
    private double overheatTime = -1;
    private boolean disabled = false;
    private List<Monitorable> motors;

    public TemperatureMonitor(List<Monitorable> motors) {
        this.motors = motors;
    }

    public void monitor() {
        boolean safe = true;
        for (Monitorable motor : motors) {
            double temp = motor.getMotorTemperature();
            if (temp > TempConstants.SAFE_TEMP) safe = false;
            if (temp > TempConstants.OVERHEAT_TEMP && overheatTime == -1) overheatTime = Timer.getFPGATimestamp();
            else if (temp > TempConstants.OVERHEAT_TEMP && Timer.getFPGATimestamp() - overheatTime >= 2) {
                disabled = true;
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
