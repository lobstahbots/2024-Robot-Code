// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;


import java.util.Arrays;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import stl.tempControl.MonitoredSparkMax;
import stl.tempControl.TemperatureMonitor;

public class ClimberSparkMax implements ClimberIO {
  private final MonitoredSparkMax leftClimberMotor;
  private final MonitoredSparkMax rightClimberMotor;
  private final RelativeEncoder leftClimberEncoder;
  private final RelativeEncoder rightClimberEncoder;
  private final TemperatureMonitor monitor;

  /*
   * Initializes a new ClimberSparkMax with a left and right climber
   * @param leftClimberID The CAN ID of the left climber.
   * @param rightClimberID The CAN ID of the right climber.
   */
  public ClimberSparkMax(int leftClimberID, int rightClimberID) {
    leftClimberMotor = new MonitoredSparkMax(leftClimberID, MotorType.kBrushless, "Left climber motor");
    rightClimberMotor = new MonitoredSparkMax(rightClimberID, MotorType.kBrushless, "Right climber motor");
    leftClimberMotor.setSmartCurrentLimit(40);
    rightClimberMotor.setSmartCurrentLimit(40);
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();
    monitor = new TemperatureMonitor(Arrays.asList(leftClimberMotor, rightClimberMotor));
  }

  /* Moves the left-side climber. */
  public void moveLeftClimber(double speed) {
    leftClimberMotor.set(speed);
  }

   /* Moves the right-side climber. */
  public void moveRightClimber(double speed) {
    rightClimberMotor.set(speed);
  }
  
  /* Stops the climber motors. */
  public void stopClimber() {
    leftClimberMotor.stopMotor();
    rightClimberMotor.stopMotor();
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftClimberVelocity = leftClimberMotor.get();
    inputs.leftClimberPosition = leftClimberEncoder.getPosition();
    inputs.leftClimberTemperature = leftClimberMotor.getMotorTemperature();
    inputs.leftClimberVoltage = leftClimberMotor.getBusVoltage() * rightClimberMotor.getAppliedOutput();
    inputs.leftClimberCurrent = leftClimberMotor.getOutputCurrent();
    inputs.rightClimberVelocity = rightClimberMotor.get();
    inputs.rightClimberPosition = rightClimberEncoder.getPosition();
    inputs.rightClimberTemperature = rightClimberMotor.getMotorTemperature();
    inputs.rightClimberVoltage = rightClimberMotor.getBusVoltage() * rightClimberMotor.getAppliedOutput(); 
    inputs.rightClimberCurrent = rightClimberMotor.getOutputCurrent();
  }

  public void periodic() {
    monitor.monitor();
  }
}
