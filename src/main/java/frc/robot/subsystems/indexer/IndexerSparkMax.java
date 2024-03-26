// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import java.util.Arrays;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IndexerConstants;
import stl.tempControl.MonitoredSparkMax;
import stl.tempControl.TemperatureMonitor;

public class IndexerSparkMax implements IndexerIO {
  private final MonitoredSparkMax indexerMotor;
  private DigitalInput intakeBeamBreak = new DigitalInput(2);
  private DigitalInput flywheelBeamBreak = new DigitalInput(1);
  private final TemperatureMonitor monitor;
  private final Debouncer debouncer = new Debouncer(IndexerConstants.DEBOUNCE_TIME, DebounceType.kBoth);

  /** Creates a new IndexerSparkMax. */
  public IndexerSparkMax(int indexerMotorID) {
    this.indexerMotor = new MonitoredSparkMax(indexerMotorID, MotorType.kBrushless, "Indexer motor");
    indexerMotor.setInverted(false);
    indexerMotor.setIdleMode(IdleMode.kBrake);
    indexerMotor.setSmartCurrentLimit(IndexerConstants.INDEXER_CURRENT_LIMIT);
    monitor = new TemperatureMonitor(Arrays.asList(indexerMotor));
  }

  public void setIndexerMotorSpeed(double indexerMotorSpeed) {
    indexerMotor.set(indexerMotorSpeed);
  }

  public void stopIndexerMotor() {
    indexerMotor.stopMotor();
  }

  public void updateInputs(IndexerIOInputs inputs) {
    inputs.indexerMotorCurrent = indexerMotor.getOutputCurrent();
    inputs.indexerMotorTemperature = indexerMotor.getMotorTemperature();
    inputs.indexerMotorVoltage = indexerMotor.getBusVoltage() * indexerMotor.getAppliedOutput();
    inputs.intakeBeamBroken = debouncer.calculate(!intakeBeamBreak.get());
    inputs.intakeBeamBrokenRaw = !intakeBeamBreak.get();
    inputs.flywheelBeamBroken = debouncer.calculate(!flywheelBeamBreak.get());
  }

  public void periodic() {
    monitor.monitor();
  }
}
  