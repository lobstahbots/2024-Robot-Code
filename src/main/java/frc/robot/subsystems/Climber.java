// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

<<<<<<< Updated upstream
=======

import org.littletonrobotics.junction.Logger;

>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

<<<<<<< Updated upstream
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
=======
  private ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /*
   * Initalizes Left and Right climber
   * Creates new climber
   * takes in two ints which are the two motor IDs
   */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  /*
   * Moves the leftside climber
   */
  public void moveLeftClimber() {
   io.moveLeftClimber();
  }

  /*
   * Moves the rightside climber
   */

  public void moveRightClimber() {
    io.moveRightClimber();
>>>>>>> Stashed changes
  }
    

  public void stopClimber() {
<<<<<<< Updated upstream
    inputs.stopClimber();
=======
    io.stopClimber();
>>>>>>> Stashed changes
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
<<<<<<< Updated upstream
=======
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
>>>>>>> Stashed changes
  }
}

