// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.DriveBase;

public class TurnToPointCommand extends TurnToAngleCommand {
  /**
   * Creates a TurnToPointCommand to turn to face a certain point.
   * @param robotPoseSupplier A supplier of the current robot position
   * @param desiredPose The desired point we want to turn to
   * @param driveBase The subsystem to control
   * @param strafeXSupplier Supplier for the X component of the robot translation.
   * @param strafeYSupplier Supplier for the Y component of the robot translation.
   * @param fieldCentric Whether the robot drives field centric. Does not affect rotation.
   */
  public TurnToPointCommand(DriveBase driveBase, Supplier<Pose2d> robotPoseSupplier, Pose2d desiredPose, DoubleSupplier strafeXSupplier, DoubleSupplier strafeYSupplier, BooleanSupplier fieldCentric) {
    super(driveBase, desiredPose.minus(robotPoseSupplier.get()).getRotation(), strafeXSupplier, strafeYSupplier, fieldCentric);
  }

  /**
  * Creates a TurnToPointCommand to turn to face a certain point.
   * @param robotPoseSupplier A supplier of the current robot position
   * @param desiredPose The desired point we want to turn to
   * @param driveBase The subsystem to control
   * @param strafeX The X component of the robot translation.
   * @param strafeY The Y component of the robot translation.
   * @param fieldCentric Whether the robot drives field centric. Does not affect rotation.
   */
  public TurnToPointCommand(DriveBase driveBase, Supplier<Pose2d> robotPoseSupplier, Pose2d desiredPose, double strafeX, double strafeY, boolean fieldCentric) {
    super(driveBase, desiredPose.minus(robotPoseSupplier.get()).getRotation(), () -> strafeX, () -> strafeY, () -> fieldCentric);
  }
}
