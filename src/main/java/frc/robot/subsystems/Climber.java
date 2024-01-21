// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private ClimberIO inputs;

  /*
   * Initalizes Left and Right climber
   * Creates new climber
   * takes in two ints which are the two motor IDs
   */
  public Climber(ClimberIO inputs) {
    this.inputs = inputs;
  }

  /*
   * Moves the leftside climber
   */
  public void moveLeftClimber() {
   inputs.moveLeftClimber();
  }

  /*
   * Moves the rightside climber
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

