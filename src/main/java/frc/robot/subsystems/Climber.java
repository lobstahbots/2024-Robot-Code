// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /*
   * Creates a new Climber.
   * @param io A ClimberIO representing the hardware layer. I
   */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  /* Moves the leftside climber. */
  public void moveLeftClimber() {
   io.moveLeftClimber();
  }

  /* Moves the rightside climber. */
  public void moveRightClimber() {
    io.moveRightClimber();
  }
    
  /*Stops the climber motors. */
  public void stopClimber() {
    io.stopClimber();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
}

