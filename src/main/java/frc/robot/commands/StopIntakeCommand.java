// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class StopIntakeCommand extends Command {
  private final Intake intake;
  
  /**
   * Command to stop {@link Intake}.
   * @param intake The intake subsystem.
   */
  public StopIntakeCommand(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
