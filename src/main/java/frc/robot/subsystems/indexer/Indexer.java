// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants.IndexerState;

public class Indexer extends SubsystemBase {
  /** Creates a new indexer. */

  private IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private IndexerState indexerState = IndexerState.NoNote;

  public Indexer(IndexerIO io) {
    this.io = io;
  }
  
  /** sets indexer speed
    @param indexerSpeed
  */
  public void setIndexerMotorSpeed(double indexerSpeed) {
    io.setIndexerMotorSpeed(indexerSpeed);
  }
  /**
  * stops indexer
  */
  public void stopIndexerMotor() {
    io.stopIndexerMotor();
  }
  
  /**
  * returns status of the beam break sensor
  * @return true if beam is broken, false if beam is not broken
  */
  public boolean intakeBeamBroken() {
    return inputs.intakeBeamBroken;
  } 
  
  /**
  * returns status of the beam break sensor
  * @return true if beam is broken, false if beam is not broken
  */
  public boolean flywheelBeamBroken() {
    return inputs.flywheelBeamBroken;
  } 

  public void setIndexerState(IndexerState newState) {
    indexerState = newState;
  }

  public IndexerState getIndexerState() {
    return indexerState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    SmartDashboard.putBoolean("Intake Beam Broken", inputs.intakeBeamBroken);
    SmartDashboard.putBoolean("Flywheel Beam Broken", inputs.flywheelBeamBroken);
    SmartDashboard.putString("State", indexerState.toString());
    io.periodic();
  }
}
