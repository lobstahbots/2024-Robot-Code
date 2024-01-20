// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.PivotConstants;

public class PivotSparkMax implements PivotIO {
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;
  private final AbsoluteEncoder encoder;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  /** Creates a new PivotReal. */
  public PivotSparkMax(int leftMotorID, int rightMotorID) {
    this.leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    this.rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    
    this.encoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

    leftMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);
    rightMotor.setSmartCurrentLimit(PivotConstants.MAX_CURRENT);

    leftMotor.follow(rightMotor, true);

    Timer.delay(0.5);
    leftMotor.burnFlash();
    Timer.delay(0.5);
    rightMotor.burnFlash();
    Timer.delay(0.5);
  }

  /**
   * Rotate both motors.
   * @param voltage The speed at which to rotate the motors.
   */
  public void setVoltage(double voltage) {
    rightMotor.setVoltage(voltage);
  }

  /**
   * Stop both motors.
   */
  public void stopPivot() {
    rightMotor.set(0);
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.position = Rotation2d.fromRotations(encoder.getPosition());

    inputs.motorLeftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.motorLeftCurrentAmps = leftMotor.getOutputCurrent();

    inputs.motorRightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.motorRightCurrentAmps = rightMotor.getOutputCurrent();
  }

  public void periodic() {
    Logger.processInputs("Pivot", inputs);
  }
}
