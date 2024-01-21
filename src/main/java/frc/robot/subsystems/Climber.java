// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private ClimberIO inputs;

  public Climber(ClimberIO inputs) {
    this.inputs = inputs;
  }

  /*
   * Logs data for the leftside climber
   */
  public void moveLeftClimber() {
   inputs.moveLeftClimber();
  }

  /*
   * logs data for the rightside climber
   */

  public void moveRightClimber() {
    inputs.moveRightClimber();
  }
    

  public void stopClimber() {
    inputs.stopClimber();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

