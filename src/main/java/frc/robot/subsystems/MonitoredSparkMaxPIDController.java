// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.TemperatureMonitor.Monitorable;

/** 
 * A temperature-monitored SPARK MAX motor PID controller. Note that in reality, this
 * is actually not a drop-in replacement for that, unlike other classes intended to
 * work with {@link TemperatureMonitor}, because those objects are not constructed
 * by the library user but returned by a method. While it would be possible to create
 * a wrapper class around a {@link SparkMaxPIDController}, but it would require
 * wrapping every single method. Thus, this simply disables the PID controller
 * (by setting its output range to (0, 0)) and doesn't make its methods
 * ineffective.
 */
public class MonitoredSparkMaxPIDController implements Monitorable {
  private boolean disabled = false;
  private CANSparkMax sparkMax;
  private SparkMaxPIDController controller;
  private double prevMin, prevMax;

  /**
   * Constructs a monitored {@link SparkMaxPIDController}.
   * @param sparkMax The SPARK MAX motor.
   * @param controller The PID controller.
   */
  public MonitoredSparkMaxPIDController(CANSparkMax sparkMax, SparkMaxPIDController controller) {
    this.sparkMax = sparkMax;
    this.controller = controller;
  }

  public void setDisabled(boolean disable) {
    if (disable && !disabled) {
      sparkMax.set(0);
      prevMax = controller.getOutputMax();
      prevMin = controller.getOutputMin();
      controller.setOutputRange(0, 0);
      Timer.delay(0.5);
      sparkMax.burnFlash();
      Timer.delay(0.5);
    } else if (!disable && disabled) {
      controller.setOutputRange(prevMin, prevMax);
      Timer.delay(0.5);
      sparkMax.burnFlash();
      Timer.delay(0.5);
    }
    disabled = disable;
  }

  public boolean getDisabled() {
    return disabled;
  }

  public double getMotorTemperature() {
    return sparkMax.getMotorTemperature();
  }
}
