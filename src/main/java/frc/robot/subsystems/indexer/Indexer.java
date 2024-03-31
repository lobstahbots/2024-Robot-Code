// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IndexerConstants.IndexerState;
import frc.robot.subsystems.leds.LEDs;

public class Indexer extends SubsystemBase {
  /** Creates a new indexer. */

  private IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final Debouncer intakeBeamBreakDebouncer = new Debouncer(IndexerConstants.DEBOUNCE_TIME, DebounceType.kBoth);
  private final Debouncer flywheelBeamBreakDebouncer = new Debouncer(IndexerConstants.DEBOUNCE_TIME, DebounceType.kBoth);
  private boolean intakeBeamBroken;
  private boolean flywheelBeamBroken;

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
    return intakeBeamBroken;
  } 
  
  /**
  * returns status of the beam break sensor
  * @return true if beam is broken, false if beam is not broken
  */
  public boolean flywheelBeamBroken() {
    return flywheelBeamBroken;
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    SmartDashboard.putBoolean("Intake Beam Broken Raw", inputs.intakeBeamBroken);
    SmartDashboard.putBoolean("Flywheel Beam Broken Raw", inputs.flywheelBeamBroken);
    SmartDashboard.putBoolean("Intake Beam Broken", intakeBeamBroken);
    SmartDashboard.putBoolean("Flywheel Beam Broken", flywheelBeamBroken);
    LEDs.getInstance().setPossession(inputs.intakeBeamBroken);
    intakeBeamBroken = intakeBeamBreakDebouncer.calculate(inputs.intakeBeamBroken);
    flywheelBeamBroken = flywheelBeamBreakDebouncer.calculate(inputs.flywheelBeamBroken);
    io.periodic();
  }

  public Command centerNoteCommand() {
    return this.run(() -> {
        if (!flywheelBeamBroken) {
            setIndexerMotorSpeed(-IndexerConstants.SLOW_INDEXER_MOTOR_SPEED);
        } else if (!intakeBeamBroken) {
            setIndexerMotorSpeed(IndexerConstants.SLOW_INDEXER_MOTOR_SPEED);
        } else {
            stopIndexerMotor();
        }
    });
  }

  public Command intakeNoteCommand(double speed) {
    return this.run(() -> {
        setIndexerMotorSpeed(speed);
    }).until(() -> flywheelBeamBroken || intakeBeamBroken);
  }
}
