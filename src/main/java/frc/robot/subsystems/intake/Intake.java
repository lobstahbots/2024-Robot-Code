// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake.
   * @param io The {@link IntakeIO} used to construct the Intake.
   */
  private IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }
 
  /**
   * Sets the intake motor speed to the given speed
   * @param intakeMotorSpeed The speed to set the motor to
   */
  public void setIntakeMotorSpeed(double intakeMotorSpeed) {
    io.setIntakeMotorSpeed(intakeMotorSpeed);
  }
  
  /** Stops the intake motor. */
  public void stopIntakeMotor() {
    io.stopIntakeMotor();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    io.periodic();
  }
}
