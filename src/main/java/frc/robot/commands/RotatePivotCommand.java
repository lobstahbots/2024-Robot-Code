// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.pivot.Pivot;

public class RotatePivotCommand extends Command {
  private final Pivot pivot;
  private final DoubleSupplier angleSupplier;

  /**
   * Create a new RotatePivotCommand.
   * @param pivot An instance of the {@link Pivot} subsystem
   * @param angleSupplier A {@link DoubleSupplier} for the angle
   */
  public RotatePivotCommand(Pivot pivot, DoubleSupplier angleSupplier) {
    this.pivot = pivot;
    this.angleSupplier = angleSupplier;
    addRequirements(pivot);
  }

  /**
   * Create a new RotatePivotCommand for a fixed angle.
   * @param pivot An instance of the {@link Pivot} subsystem
   * @param angle The angle to rotate the pivot to
   */
  public RotatePivotCommand(Pivot pivot, double angle) {
    this(pivot, () -> angle);
  }

  @Override
  public void execute() {
    pivot.setDesiredAngle(MathUtil.clamp(angleSupplier.getAsDouble(), 0, PivotConstants.PIVOT_MAX_ANGLE_DEGREES));
    // pivot.runVolts(10);
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
