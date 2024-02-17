// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClimberSparkMax implements ClimberIO {
  public final CANSparkMax leftClimberMotor;
  public final CANSparkMax rightClimberMotor;
  public final RelativeEncoder leftClimberEncoder;
  public final RelativeEncoder rightClimberEncoder;

  /*
   * Initializes a new ClimberSparkMax with a left and right climber
   * @param leftClimberID The CAN ID of the left climber.
   * @param rightClimberID The CAN ID of the right climber.
   */
  public ClimberSparkMax(int leftClimberID, int rightClimberID) {
    this.leftClimberMotor = new CANSparkMax(leftClimberID, MotorType.kBrushless);
    this.rightClimberMotor = new CANSparkMax(rightClimberID, MotorType.kBrushless);
    this.leftClimberMotor.setSmartCurrentLimit(40);
    this.rightClimberMotor.setSmartCurrentLimit(40);
    this.leftClimberMotor.setIdleMode(IdleMode.kBrake);
    this.leftClimberMotor.setIdleMode(IdleMode.kBrake);
    this.leftClimberEncoder = leftClimberMotor.getEncoder();
    this.rightClimberEncoder = rightClimberMotor.getEncoder();
  }

  /* Moves the leftside climber. */
  public void moveLeftClimber(double speed) {
    leftClimberMotor.set(speed);
  }

   /* Moves the rightside climber. */
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
}
