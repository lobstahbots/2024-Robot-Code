// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import java.util.Arrays;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.PivotConstants;
import stl.tempControl.MonitoredSparkMax;
import stl.tempControl.TemperatureMonitor;

public class PivotSparkMax implements PivotIO {
  private final MonitoredSparkMax leftMotor;
  private final MonitoredSparkMax rightMotor;
  private final DutyCycleEncoder encoder;
  private final TemperatureMonitor monitor;

  /** Creates a new PivotSparkMax. 
   * 
   * @param leftMotorID The CAN ID of the left motor.
   * @param rightMotorID The CAN ID of the right motor.
   * @param pivotEncoderChannel The channel for the arm encoder to plug into the RIO.
   */
  public PivotSparkMax(int leftMotorID, int rightMotorID, int pivotEncoderChannel) {
    this.leftMotor = new MonitoredSparkMax(leftMotorID, MotorType.kBrushless, "Left pivot motor");
    this.rightMotor = new MonitoredSparkMax(rightMotorID, MotorType.kBrushless, "Right pivot motor");
    
    this.encoder = new DutyCycleEncoder(new DigitalInput(pivotEncoderChannel));

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);
    rightMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);

    rightMotor.setInverted(false);
    leftMotor.follow(rightMotor, true);

    Timer.delay(0.5);
    leftMotor.burnFlash();
    Timer.delay(0.5);
    rightMotor.burnFlash();
    Timer.delay(0.5);

    monitor = new TemperatureMonitor(Arrays.asList(leftMotor, rightMotor));
  }

  /**
   * Sets voltage of both motors.
   * @param voltage The speed at which to rotate the motors.
   */
  public void setVoltage(double voltage) {
    rightMotor.setVoltage(voltage);
  }

  /**
   * Stops both motors.
   */
  public void stopPivot() {
    rightMotor.set(0);
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.position = Rotation2d.fromRotations(encoder.getAbsolutePosition());

    inputs.motorLeftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.motorLeftCurrentAmps = leftMotor.getOutputCurrent();

    inputs.motorRightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.motorRightCurrentAmps = rightMotor.getOutputCurrent();
  }

  public void periodic() {
    monitor.monitor();
  }
}
