// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public Rotation2d motorLeftRotation = new Rotation2d();
    public double motorLeftAppliedVolts = 0.0;
    public double motorLeftCurrentAmps = 0.0;

    public Rotation2d motorRightRotation = new Rotation2d();
    public double motorRightAppliedVolts = 0.0;
    public double motorRightCurrentAmps = 0.0;
  }

  public default void updateInputs(PivotIOInputs inputs) {}

  /**
   * Rotate both motors.
   * @param speed The speed at which to rotate the motors.
   */
  public default void rotatePivot(double speed) {}

  /** Stop both motors.
   */
  public default void stopPivot() {}
}
