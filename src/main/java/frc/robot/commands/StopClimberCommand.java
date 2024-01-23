// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class StopClimberCommand extends Command {
  public final Climber climber;

  /** Creates a new StopClimberCommand. */
  public StopClimberCommand(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.stopClimber();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}