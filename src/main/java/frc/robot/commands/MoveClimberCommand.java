// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class MoveClimberCommand extends Command {
  private final Climber climber;
  private final double climberSpeed; 

  /** Creates a new MoveClimberCommand. */
  public MoveClimberCommand(Climber climber, double climberSpeed) {
    this.climber = climber;
    this.climberSpeed = climberSpeed;
    addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.moveLeftClimber(climberSpeed);
    climber.moveRightClimber(climberSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    climber.stopClimber();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
