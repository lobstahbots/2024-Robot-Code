// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private PivotIO io;
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
  public Pivot(PivotIO io) {
    this.io = io;
  }

  /**
   * Stop the pivot.
   */
  public void stopPivot() {
    io.stopPivot();
  }

  /**
   * Set the desired angle (in radians).
   * @param angle The desired angle
   */
  public void setDesiredAngle(double angle) {
    double pidOutput = controller.calculate(inputs.position.getRadians(), angle);
    controller.setGoal(inputs.position.getRadians());
    State setpoint = controller.getSetpoint();
    double feedforwardOutput = feedforward.calculate(setpoint.position, setpoint.velocity);
    io.setVoltage(pidOutput + feedforwardOutput);
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

  public void setIdleMode(IdleMode idleMode) {
    io.setIdleMode();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    io.periodic();
  }
}
