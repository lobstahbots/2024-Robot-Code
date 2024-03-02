// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PivotConstants;
import stl.sysId.CharacterizableSubsystem;

public class Pivot extends CharacterizableSubsystem {
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
    controller.setGoal(0);
    controller.setTolerance(1);
  }

  /**
   * Stop the pivot.
   */
  public void stopPivot() {
    io.stopPivot();
  }

  /**
   * Set the desired angle (in degrees).
   * @param angle The desired angle
   */
  public void setDesiredAngle(double angle) {
    double pidOutput = controller.calculate(inputs.position.getDegrees(), angle);
    controller.setGoal(inputs.position.getDegrees());
    State setpoint = controller.getGoal();
    double feedforwardOutput = feedforward.calculate(setpoint.position, setpoint.velocity);
    io.setVoltage(pidOutput + feedforwardOutput);
  }

  /**
   * Reset the PID controller error.
   */
  public void resetControllerError() {
    controller.reset(inputs.position.getDegrees());
  }

  /** Gets pivot rotation. */
  public Rotation2d getPosition() {
    return inputs.position;
  }

  public void setIdleMode(IdleMode idleMode) {
    io.setIdleMode();
  }

  @Override
  /**Runs pivot motors during characterization voltage ramp routines.*/
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }


  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    io.periodic();
    Logger.recordOutput("TowerPose",
        new Pose3d(PivotConstants.ORIGIN_TO_TOWER_MOUNT_X_DIST, PivotConstants.ORIGIN_TO_TOWER_MOUNT_Y_DIST,
            PivotConstants.ORIGIN_TO_TOWER_MOUNT_Z_DIST, PivotConstants.TOWER_ROTATION));
    Pose3d pivotPose3d = new Pose3d(0.0,
        0.0, 0.0,
        new Rotation3d(0, PivotConstants.ARM_PITCH,
            PivotConstants.ARM_YAW))
        .exp(new Twist3d(0.0, 0.0, 0.0, PivotConstants.ARM_INITIAL_ROLL + inputs.position.getRadians(), 0, 0))
        .exp(new Twist3d(0.0, PivotConstants.ORIGIN_TO_ARM_MOUNT_Y_DIST, PivotConstants.ORIGIN_TO_ARM_MOUNT_Z_OFFSET_DIST, 0.0, 0.0, 0.0));
      pivotPose3d = new Pose3d(pivotPose3d.getX() + PivotConstants.ORIGIN_TO_ARM_MOUNT_X_DIST, pivotPose3d.getY(), pivotPose3d.getZ() + PivotConstants.ORIGIN_TO_ARM_MOUNT_Z_DIST, pivotPose3d.getRotation());
        Logger.recordOutput("ArmPose", pivotPose3d);
    SmartDashboard.putNumber("Arm encoder value", inputs.position.getDegrees());
  }
}
