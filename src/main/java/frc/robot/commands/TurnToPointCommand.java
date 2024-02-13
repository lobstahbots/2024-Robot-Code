// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.drive.DriveBase;

public class TurnToPointCommand extends TurnToAngleCommand {
  /**
   * Creates a TurnToPointCommand to turn to face a certain point.
   * @param robotPoseSupplier A supplier of the current robot position
   * @param desiredPose The desired point we want to turn to
   * @param driveBase The subsystem to control
   */

  public TurnToPointCommand(Supplier<Pose2d> robotPoseSupplier, Pose2d desiredPose, DriveBase driveBase) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(driveBase, desiredPose.minus(robotPoseSupplier.get()).getRotation());
  }
}
