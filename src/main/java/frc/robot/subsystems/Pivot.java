// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;;

public class Pivot extends SubsystemBase {
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private PivotSparkMax hardware;
  private final ArmFeedforward feedforward = new ArmFeedforward(
    PivotConstants.KS, PivotConstants.KV, PivotConstants.KA
  );
  /** Creates a new Pivot. */
  public Pivot(PivotSparkMax hardware) {
    this.hardware = hardware;
  }

  /**
   * Stop the pivot.
   */
  public void stopPivot() {
    hardware.stopPivot();
  }

  /**
   * Rotate the pivot at speed `speed`, which is a duty-cycle value (in [-1, 1]).
   * @param speed
   */
  public void rotatePivot(double speed) {
    hardware.rotatePivot(
      feedforward.calculate(hardware.getLeftPositionRadians(), speed),
      feedforward.calculate(hardware.getRightPositionRadians(), speed)
    );
  }

  @Override
  public void periodic() {
    hardware.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
  }
}
