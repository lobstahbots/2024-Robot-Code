// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ClimberSparkMax extends SubsystemBase implements ClimberIO{
  public final CANSparkMax leftClimber;
  public final CANSparkMax rightClimber;
  /*
   * Initalizes Left and Right climber
   * Creates new climber
   * takes in two ints which are the two motor IDs
   */
  public ClimberSparkMax(int leftClimberID, int rightClimberID) {
    this.leftClimber = new CANSparkMax(leftClimberID, MotorType.kBrushless);
    this.rightClimber = new CANSparkMax(rightClimberID, MotorType.kBrushless);
    this.leftClimber.setSmartCurrentLimit(40);
    this.rightClimber.setSmartCurrentLimit(40);
  }
  /*
   * Moves the leftside climber
   */
  
  public void moveLeftClimber() {
    leftClimber.set(RobotConstants.CLIMBER_SPEED);
  }

  /*
   * Moves the rightside climber
   */

  public void moveRightClimber() {
    rightClimber.set(RobotConstants.CLIMBER_SPEED);
  }

  public void stopClimber() {
    leftClimber.stopMotor();
    rightClimber.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
