// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class RotatePivotCommand extends Command {
  private final Pivot pivot;
  private final double angle;

  /**
   * Create a new RotatePivotCommand.
   * @param pivot An instance of the {@link Pivot} subsystem
   * @param angle The angle in radians to pivot to
   */
  public RotatePivotCommand(Pivot pivot, double angle) {
    this.pivot = pivot;
    this.angle = angle;
    addRequirements(pivot);
  }

  @Override
  public void execute() {
    pivot.setDesiredAngle(angle);
  }

  @Override
  public void end(boolean interrupted) {
    pivot.stopPivot();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
