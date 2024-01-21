// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIOInputs io) {
    this.io = io;
  }

  public void setIntakeMotorSpeed(double intakeMotorSpeed) {
    io.setIntakeMotorSpeed(intakeMotorSpeed);
  }
  
  public void stopIntakeMotor() {
    io.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
