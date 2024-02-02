// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.subsystems.TemperatureMonitor.Monitorable;

/** A temperature-monitored SPARK MAX motor controller. */
public class MonitoredSparkMax extends CANSparkMax implements Monitorable {
    private boolean disabled = false;
    /**
   * Create a new object to control a SPARK MAX motor Controller
   *
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected
   *     to their matching colors and the hall sensor must be plugged in. Brushed motors must be
   *     connected to the Red and Black terminals only.
   */
  public MonitoredSparkMax(int deviceId, MotorType type) {
    super(deviceId, type);
  }

  public void setDisabled(boolean disable) {
    disabled = disable;
  }

  public boolean getDisabled() {
    return disabled;
  }

  public void set(double speed) {
    if (!disabled) super.set(speed);
  }

  public void setVoltage(double outputVolts) {
    if (!disabled) super.setVoltage(outputVolts);
  }
}
