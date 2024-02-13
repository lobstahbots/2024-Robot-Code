// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Arrays;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import stl.math.LobstahMath;
import stl.motorcontrol.MonitoredSparkMax;
import stl.motorcontrol.TemperatureMonitor;

public class SwerveModuleReal implements SwerveModuleIO {
  private final MonitoredSparkMax angleMotor;
  private final MonitoredSparkMax driveMotor;
  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder angleAbsoluteEncoder;
  private Rotation2d angularOffset;
  private final int moduleID;
  private final TemperatureMonitor monitor;

  /** Creates a new SwerveModule for real cases. 
   * 
   * @param moduleID The module number (0-3).
   * @param angleMotorID The CAN ID of the motor controlling the angle.
   * @param driveMotorID The CAN ID of the motor controlling drive speed.
   * @param angularOffsetDegrees The offset angle in degrees.
  */
  public SwerveModuleReal (int moduleID, int angleMotorID, int driveMotorID, double angularOffsetDegrees, boolean inverted) {
    this.moduleID = moduleID;

    this.angleMotor = new MonitoredSparkMax(angleMotorID, MotorType.kBrushless);
    this.driveMotor = new MonitoredSparkMax(driveMotorID, MotorType.kBrushless);

    angleMotor.restoreFactoryDefaults();
    driveMotor.restoreFactoryDefaults();
    driveMotor.setIdleMode(IdleMode.kBrake);
    angleMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setSmartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    angleMotor.setSmartCurrentLimit(DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT);
    driveMotor.enableVoltageCompensation(12.0);
    angleMotor.enableVoltageCompensation(12.0);
    angleMotor.setInverted(inverted);

    drivingEncoder = driveMotor.getEncoder();
    angleAbsoluteEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);

    angleAbsoluteEncoder.setPositionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_CONVERSION_FACTOR);
    angleAbsoluteEncoder.setVelocityConversionFactor(SwerveConstants.TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR);

    drivingEncoder.setPositionConversionFactor(SwerveConstants.DRIVING_ENCODER_POSITION_CONVERSION_FACTOR);
    drivingEncoder.setVelocityConversionFactor(SwerveConstants.DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR);

    Timer.delay(0.5);
    driveMotor.burnFlash();
    Timer.delay(0.5);
    angleMotor.burnFlash();
    Timer.delay(0.5);

    monitor = new TemperatureMonitor(Arrays.asList(driveMotor, angleMotor));

    this.angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees);
    drivingEncoder.setPosition(0);
  }

  /**Stops the module motors. */
  public void stopMotors() {
    angleMotor.stopMotor();
    driveMotor.stopMotor();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current encoder velocities of the module as a {@link SwerveModuleState}.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(drivingEncoder.getVelocity(), new Rotation2d(angleAbsoluteEncoder.getPosition() + angularOffset.getRadians()));
  }
  

  /**
   * Returns the module ID.
   *
   * @return The ID number of the module (0-3).
   */
  public int getModuleID() {
    return moduleID;
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current encoder positions position of the module as a {@link SwerveModulePosition}.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(drivingEncoder.getPosition(), new Rotation2d(angleAbsoluteEncoder.getPosition() + angularOffset.getRadians()));
  }

  /**
   * Sets the braking mode of the driving motor.
   * 
   * @param the {@link IdleMode} to set motors to.
   */
  public void setDriveBrakingMode(IdleMode mode) {
    driveMotor.setIdleMode(mode);
  }

   /**
   * Sets the braking mode of the turning motor.
   * 
   * @param the {@link IdleMode} to set motors to.
   */
  public void setTurnBrakingMode(IdleMode mode) {
    angleMotor.setIdleMode(mode);
  }

   /** Zeroes the drive encoder. */
   public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }

  /**Sets voltage of driving motor.
   * 
   * @param volts The voltage the motor should be set to.
   */
  public void setDriveVoltage(double volts) {
    driveMotor.setVoltage(volts);
  }

  /**Sets voltage of turn motor.
   * 
   * @param volts The voltage the motor should be set to.
   */
  public void setTurnVoltage(double volts) {
    angleMotor.setVoltage(volts);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePosition = Rotation2d.fromRotations(drivingEncoder.getPosition());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(angleAbsoluteEncoder.getVelocity() / 60);
    inputs.driveAppliedVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveMotor.getOutputCurrent()};

    inputs.turnPosition = Rotation2d.fromRadians(LobstahMath.wrapValue(angleAbsoluteEncoder.getPosition() + angularOffset.getRadians(), 0, 2*Math.PI));
    inputs.turnAppliedVolts = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {angleMotor.getOutputCurrent()};
    inputs.angularOffset = angularOffset;
  }

  public void periodic() {
    monitor.monitor();
  }

}
