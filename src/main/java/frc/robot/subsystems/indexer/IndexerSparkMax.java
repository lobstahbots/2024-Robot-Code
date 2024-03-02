// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IndexerConstants;

public class IndexerSparkMax implements IndexerIO {
  private final CANSparkMax indexerMotor;
  private DigitalInput beamBreak = new DigitalInput(0);

  /** Creates a new IndexerSparkMax. */
  public IndexerSparkMax(int indexerMotorID) {
    this.indexerMotor = new CANSparkMax(indexerMotorID, MotorType.kBrushless);
    this.indexerMotor.setInverted(false);
    this.indexerMotor.setIdleMode(IdleMode.kBrake);
    this.indexerMotor.setSmartCurrentLimit(IndexerConstants.INDEXER_CURRENT_LIMIT);
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
    inputs.beamBroken = !beamBreak.get();
  } 
}
