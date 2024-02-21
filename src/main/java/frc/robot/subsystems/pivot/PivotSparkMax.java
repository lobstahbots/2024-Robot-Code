// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.PivotConstants;
import stl.tempControl.MonitoredSparkMax;
import stl.tempControl.TemperatureMonitor;

public class PivotSparkMax implements PivotIO {
  private final MonitoredSparkMax leftMotor;
  private final MonitoredSparkMax rightMotor;
  private final AbsoluteEncoder encoder;
  private final TemperatureMonitor monitor;

  /** Creates a new PivotSparkMax. 
   * 
   * @param leftMotorID The CAN ID of the left motor.
   * @param rightMotorID The CAN ID of the right motor.
   */
  public PivotSparkMax(int leftMotorID, int rightMotorID) {
    this.leftMotor = new MonitoredSparkMax(leftMotorID, MotorType.kBrushless, "Left pivot motor");
    this.rightMotor = new MonitoredSparkMax(rightMotorID, MotorType.kBrushless, "Right pivot motor");
    
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
    inputs.position = Rotation2d.fromRotations(encoder.getPosition());
    inputs.velocity = Rotation2d.fromRotations(encoder.getVelocity());

    inputs.motorLeftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.motorLeftCurrentAmps = leftMotor.getOutputCurrent();

    inputs.motorRightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.motorRightCurrentAmps = rightMotor.getOutputCurrent();
  }

  public void periodic() {
    monitor.monitor();
    var pivotAngleRads = Rotation2d.fromRotations(encoder.getPosition()).getRadians();
    var logTransform = PivotKinematics.angleToSimPivotTransform(pivotAngleRads);
    Logger.recordOutput("TowerPose", new Pose3d(PivotConstants.ORIGIN_TO_TOWER_MOUNT_X_DIST, PivotConstants.ORIGIN_TO_TOWER_MOUNT_Y_DIST, PivotConstants.ORIGIN_TO_TOWER_MOUNT_Z_DIST, PivotConstants.TOWER_ROTATION));
    Logger.recordOutput("ArmPose", new Pose3d(PivotConstants.ORIGIN_TO_ARM_MOUNT_X_DIST, PivotConstants.ORIGIN_TO_ARM_MOUNT_Y_DIST, PivotConstants.ORIGIN_TO_ARM_MOUNT_Z_DIST + logTransform.getY(), new Rotation3d(PivotConstants.ARM_INITIAL_ROLL - pivotAngleRads, PivotConstants.ARM_PITCH, PivotConstants.ARM_YAW)));
  }
}
