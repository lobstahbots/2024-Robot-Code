// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PivotConstants;

public class PivotSparkMax implements PivotIO {
  private final MonitoredSparkMax leftMotor;
  private final MonitoredSparkMax rightMotor;
  private final AbsoluteEncoder encoder;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final TemperatureMonitor monitor;

  private final Mechanism2d pivot = new Mechanism2d(2, 11.855);
  private final MechanismRoot2d root = pivot.getRoot("pivot", 14, 1);
  private final MechanismLigament2d arm;
  private final MechanismLigament2d shooter;

  /** Creates a new PivotReal. */
  public PivotSparkMax(int leftMotorID, int rightMotorID) {
    this.leftMotor = new MonitoredSparkMax(leftMotorID, MotorType.kBrushless);
    this.rightMotor = new MonitoredSparkMax(rightMotorID, MotorType.kBrushless);
    this.arm = root.append(new MechanismLigament2d("arm", 16, 45));
    this.shooter = root.append(new MechanismLigament2d("shooter", 7.5, 45));
    
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
    arm.setAngle(Rotation2d.fromRotations(encoder.getPosition()));
    SmartDashboard.putData("Pivot", pivot);
  }
}
