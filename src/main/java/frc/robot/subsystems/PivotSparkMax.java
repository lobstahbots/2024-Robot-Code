// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.PivotConstants;

public class PivotSparkMax implements PivotIO {
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;
  private final AbsoluteEncoder leftEncoder;
  private final AbsoluteEncoder rightEncoder;
  private final SparkMaxPIDController leftController;
  private final SparkMaxPIDController rightController;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  /** Creates a new PivotReal. */
  public PivotSparkMax(int leftMotorID, int rightMotorID) {
    this.leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    this.rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    
    this.leftEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);
    this.rightEncoder = rightMotor.getAbsoluteEncoder(Type.kDutyCycle);

    this.leftController = leftMotor.getPIDController();
    leftController.setP(PivotConstants.PID_P);
    leftController.setI(PivotConstants.PID_I);
    leftController.setD(PivotConstants.PID_D);
    leftController.setFF(PivotConstants.PID_FF);
    this.rightController = rightMotor.getPIDController();
    rightController.setP(PivotConstants.PID_P);
    rightController.setI(PivotConstants.PID_I);
    rightController.setD(PivotConstants.PID_D);
    rightController.setFF(PivotConstants.PID_FF);

    Timer.delay(0.5);
    leftMotor.burnFlash();
    Timer.delay(0.5);
    rightMotor.burnFlash();
    Timer.delay(0.5);
  }

  /**
   * Rotate both motors.
   * @param speed The speed at which to rotate the motors.
   */
  public void rotatePivot(double speed) {
    leftController.setReference(speed, ControlType.kDutyCycle);
    rightController.setReference(speed, ControlType.kDutyCycle);
  }

  /** Stop both motors.
   */
  public void stopPivot() {
    leftController.setReference(0, ControlType.kVelocity);
    rightController.setReference(0, ControlType.kVelocity);
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.motorLeftRotation = Rotation2d.fromRotations(leftEncoder.getPosition());
    inputs.motorLeftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.motorLeftCurrentAmps = leftMotor.getOutputCurrent();

    inputs.motorRightRotation = Rotation2d.fromRotations(rightEncoder.getPosition());
    inputs.motorRightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.motorRightCurrentAmps = rightMotor.getOutputCurrent();
  }

  public void periodic() {
    Logger.processInputs("Pivot", inputs);
  }
}
