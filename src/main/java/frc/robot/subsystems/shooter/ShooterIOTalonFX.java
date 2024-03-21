// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.Arrays;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ShooterConstants;
import stl.tempControl.MonitoredTalonFX;
import stl.tempControl.TemperatureMonitor;

public class ShooterIOTalonFX implements ShooterIO {
  private final MonitoredTalonFX upperShooterMotor;
  private final MonitoredTalonFX lowerShooterMotor;
  private final TemperatureMonitor monitor;

  /**
   * Creates a new Shooter subsystem.
   * @param upperShooterMotorID The CAN ID of the upper shooter motor
   * @param lowerShooterMotorID The CAN ID of the lower shooter motor
   */
  public ShooterIOTalonFX(int upperShooterMotorID, int lowerShooterMotorID) {
    this.upperShooterMotor = new MonitoredTalonFX(upperShooterMotorID, "Upper shooter motor");
    this.lowerShooterMotor = new MonitoredTalonFX(lowerShooterMotorID, "Lower shooter motor");
    this.upperShooterMotor.setInverted(true);
    this.lowerShooterMotor.setInverted(true);
    this.upperShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    this.lowerShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    this.upperShooterMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT));
    this.lowerShooterMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT));
    
    this.monitor = new TemperatureMonitor(Arrays.asList(upperShooterMotor, lowerShooterMotor));
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

  public void setIdleMode(NeutralModeValue idleMode) {
    upperShooterMotor.setNeutralMode(idleMode);
    lowerShooterMotor.setNeutralMode(idleMode);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.upperShooterMotorVoltage = upperShooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.upperShooterMotorTemperature = upperShooterMotor.getDeviceTemp().getValueAsDouble();
    inputs.upperShooterMotorCurrent = upperShooterMotor.getSupplyCurrent().getValueAsDouble();
    inputs.upperShooterMotorVelocity = upperShooterMotor.getVelocity().getValueAsDouble();
    inputs.lowerShooterMotorVoltage = lowerShooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.lowerShooterMotorTemperature = lowerShooterMotor.getDeviceTemp().getValueAsDouble();
    inputs.lowerShooterMotorCurrent = lowerShooterMotor.getSupplyCurrent().getValueAsDouble();
    inputs.lowerShooterMotorVelocity = lowerShooterMotor.getVelocity().getValueAsDouble();
  }

  public void periodic() {
    monitor.monitor();
  }
}
