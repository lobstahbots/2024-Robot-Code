// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< Updated upstream
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ClimberSparkMax extends SubsystemBase implements ClimberIO{
  public final CANSparkMax leftClimber;
  public final CANSparkMax rightClimber;
=======

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.RobotConstants;

public class ClimberSparkMax implements ClimberIO{
  public final CANSparkMax leftClimberMotor;
  public final CANSparkMax rightClimberMotor;
  public final RelativeEncoder leftClimberEncoder;
  public final RelativeEncoder rightClimberEncoder;
>>>>>>> Stashed changes
  /*
   * Initalizes Left and Right climber
   * Creates new climber
   * takes in two ints which are the two motor IDs
   */
  public ClimberSparkMax(int leftClimberID, int rightClimberID) {
<<<<<<< Updated upstream
    this.leftClimber = new CANSparkMax(leftClimberID, MotorType.kBrushless);
    this.rightClimber = new CANSparkMax(rightClimberID, MotorType.kBrushless);
    this.leftClimber.setSmartCurrentLimit(40);
    this.rightClimber.setSmartCurrentLimit(40);
=======
    this.leftClimberMotor = new CANSparkMax(leftClimberID, MotorType.kBrushless);
    this.rightClimberMotor = new CANSparkMax(rightClimberID, MotorType.kBrushless);
    this.leftClimberMotor.setSmartCurrentLimit(40);
    this.rightClimberMotor.setSmartCurrentLimit(40);
    this.leftClimberEncoder = leftClimberMotor.getEncoder();
    this.rightClimberEncoder = rightClimberMotor.getEncoder();
>>>>>>> Stashed changes
  }
  /*
   * Moves the leftside climber
   */
  
  public void moveLeftClimber() {
<<<<<<< Updated upstream
    leftClimber.set(RobotConstants.CLIMBER_SPEED);
=======
    leftClimberMotor.set(RobotConstants.CLIMBER_SPEED);
>>>>>>> Stashed changes
  }

  /*
   * Moves the rightside climber
   */

  public void moveRightClimber() {
<<<<<<< Updated upstream
    rightClimber.set(RobotConstants.CLIMBER_SPEED);
  }

  public void stopClimber() {
    leftClimber.stopMotor();
    rightClimber.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
=======
    rightClimberMotor.set(RobotConstants.CLIMBER_SPEED);
  }

  public void stopClimber() {
    leftClimberMotor.stopMotor();
    rightClimberMotor.stopMotor();
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftClimberVelocity = leftClimberMotor.get();
    inputs.leftClimberPosition = leftClimberEncoder.getPosition();
    inputs.leftClimberTemperature = leftClimberMotor.getMotorTemperature();
    inputs.leftClimberVoltage = leftClimberMotor.getBusVoltage();
    inputs.rightClimberVelocity = rightClimberMotor.get();
    inputs.rightClimberPosition = rightClimberEncoder.getPosition();
    inputs.rightClimberTemperature = rightClimberMotor.getMotorTemperature();
    inputs.rightClimberVoltage = rightClimberMotor.getBusVoltage();
>>>>>>> Stashed changes
  }
}
