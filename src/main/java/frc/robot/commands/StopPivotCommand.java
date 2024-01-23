// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class StopPivotCommand extends Command {
  private final Pivot pivot;

  /**
   * Create a new StopPivotCommand.
   * @param pivot An instance of the {@link Pivot} subsystem
   */
  public StopPivotCommand(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  @Override
  public void execute() {
    pivot.stopPivot();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
