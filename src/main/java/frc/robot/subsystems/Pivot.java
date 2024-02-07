// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private PivotSparkMax pivotMotor;
  private final ArmFeedforward feedforward = new ArmFeedforward(
    PivotConstants.KS, PivotConstants.KV, PivotConstants.KA
  );
  private final ProfiledPIDController controller = new ProfiledPIDController(
    PivotConstants.PID_P,
    PivotConstants.PID_I,
    PivotConstants.PID_D,
    new TrapezoidProfile.Constraints(PivotConstants.MAX_VELOCITY, PivotConstants.MAX_ACCELERATION)
  );
  /** Creates a new Pivot. */
  public Pivot(PivotSparkMax pivotMotor) {
    this.pivotMotor = pivotMotor;
  }

  /**
   * Stop the pivot.
   */
  public void stopPivot() {
    pivotMotor.stopPivot();
  }

  /**
   * Set the desired angle (in radians).
   * @param angle The desired angle
   */
  public void setDesiredAngle(double angle) {
    double pidOutput = controller.calculate(inputs.position.getRadians(), angle);
    State setpoint = controller.getSetpoint();
    double feedforwardOutput = feedforward.calculate(setpoint.position, setpoint.velocity);
    pivotMotor.setVoltage(pidOutput + feedforwardOutput);
  }

  /**
   * Reset the PID controller error.
   */
  public void resetControllerError() {
    controller.reset(inputs.position.getRadians());
  }

  public Rotation2d getPosition() {
    return inputs.position;
  }

  @Override
  public void periodic() {
    pivotMotor.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    pivotMotor.periodic();
  }
}
