// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.Arrays;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import stl.tempControl.MonitoredSparkMax;
import stl.tempControl.TemperatureMonitor;

public class ShooterSparkMax implements ShooterIO {
  private final MonitoredSparkMax upperShooterMotor;
  private final MonitoredSparkMax lowerShooterMotor;
  private final TemperatureMonitor monitor;

  /**
   * Creates a new Shooter subsystem.
   * @param upperShooterMotorID The CAN ID of the upper shooter motor
   * @param lowerShooterMotorID The CAN ID of the lower shooter motor
   */
  public ShooterSparkMax(int upperShooterMotorID, int lowerShooterMotorID) {
    this.upperShooterMotor = new MonitoredSparkMax(upperShooterMotorID, MotorType.kBrushless);
    this.lowerShooterMotor = new MonitoredSparkMax(lowerShooterMotorID, MotorType.kBrushless);
    this.upperShooterMotor.setInverted(true);
    this.lowerShooterMotor.setInverted(false);
    this.upperShooterMotor.setIdleMode(IdleMode.kBrake);
    this.lowerShooterMotor.setIdleMode(IdleMode.kBrake);
    this.upperShooterMotor.setSmartCurrentLimit(40);
    this.lowerShooterMotor.setSmartCurrentLimit(40);

    monitor = new TemperatureMonitor(Arrays.asList(upperShooterMotor, lowerShooterMotor));
  }

  /**
   * Spins the shooter motors at the given speeds.
   * @param upperShooterSpeed the speed to set the upper shooter to
   * @param lowerShooterSpeed the speed to set the lower shooter to
   
   */
  public void setShooterSpeed(double upperShooterSpeed, double lowerShooterSpeed) {
    upperShooterMotor.set(upperShooterSpeed);
    lowerShooterMotor.set(lowerShooterSpeed);
  }

  /**
   * Stops the shooter motors.
   */
  public void stopMotor() {
    upperShooterMotor.stopMotor();
    lowerShooterMotor.stopMotor();
  }
  public void updateInputs(ShooterIOInputs inputs) {
      inputs.upperShooterMotorVoltage = upperShooterMotor.getBusVoltage() * upperShooterMotor.getAppliedOutput();
      inputs.upperShooterMotorTemperature = upperShooterMotor.getMotorTemperature();
      inputs.upperShooterMotorCurrent = upperShooterMotor.getOutputCurrent();
      inputs.upperShooterMotorVelocity = upperShooterMotor.getEncoder().getVelocity();
      inputs.lowerShooterMotorVoltage = lowerShooterMotor.getBusVoltage() * lowerShooterMotor.getAppliedOutput();
      inputs.lowerShooterMotorTemperature = lowerShooterMotor.getMotorTemperature();
      inputs.lowerShooterMotorCurrent = lowerShooterMotor.getOutputCurrent();
      inputs.lowerShooterMotorVelocity = lowerShooterMotor.getEncoder().getVelocity();
    }
  public void periodic() {
    monitor.monitor();
  }
}
