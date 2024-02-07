// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.PivotConstants;

public class PivotSparkMax implements PivotIO {
  private final MonitoredSparkMax leftMotor;
  private final MonitoredSparkMax rightMotor;
  private final AbsoluteEncoder encoder;
  private final TemperatureMonitor monitor;

  /** Creates a new PivotReal. */
  public PivotSparkMax(int leftMotorID, int rightMotorID) {
    this.leftMotor = new MonitoredSparkMax(leftMotorID, MotorType.kBrushless);
    this.rightMotor = new MonitoredSparkMax(rightMotorID, MotorType.kBrushless);
    
    this.encoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);

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
    inputs.velocity = Rotation2d.fromRotations(encoder.getVelocity());

    inputs.motorLeftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.motorLeftCurrentAmps = leftMotor.getOutputCurrent();

    inputs.motorRightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.motorRightCurrentAmps = rightMotor.getOutputCurrent();
  }

  public void periodic() {
    monitor.monitor();
    Logger.recordOutput("ArmPose", new Pose3d(Units.inchesToMeters(16.5),Units.inchesToMeters(8.65), Units.inchesToMeters(3.375), new Rotation3d(Units.degreesToRadians(90), 0, Units.degreesToRadians(90))));
    Logger.recordOutput("IntakePose", new Pose3d(Units.inchesToMeters(8 + Rotation2d.fromRotations(encoder.getPosition()).getRadians()),Units.inchesToMeters(0), Units.inchesToMeters(5.25 - 7*(Rotation2d.fromRotations(encoder.getPosition()).getRadians())), new Rotation3d(Units.degreesToRadians(-45) - Rotation2d.fromRotations(encoder.getPosition()).getRadians(), 0, Units.degreesToRadians(90))));
  }
}
