// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PivotConstants;
import stl.trajectory.AlliancePoseMirror;

import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  private static final Translation3d blueSpeaker = FieldConstants.BLUE_ALLIANCE_SPEAKER_POSE3D.getTranslation();
  private static Supplier<Transform3d> launcherTransform = () -> new Transform3d();
  private static final double shotSpeed = 5.0; // Meters per sec
  private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  public static void setRobotPoseSupplier(Supplier<Pose2d> supplier, Supplier<Rotation2d> launchAngleSupplier) {
    robotPoseSupplier = supplier;
    launcherTransform = () -> new Transform3d(PivotConstants.ORIGIN_TO_ARM_MOUNT_X_DIST, PivotConstants.ORIGIN_TO_ARM_MOUNT_Y_DIST, PivotConstants.ORIGIN_TO_ARM_MOUNT_Z_DIST, new Rotation3d(Units.degreesToRadians(launchAngleSupplier.get().getDegrees() + 15), PivotConstants.ARM_PITCH, PivotConstants.ARM_YAW));
  }

  public static Command shoot() {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  final Pose3d startPose =
                      new Pose3d(robotPoseSupplier.get()).transformBy(launcherTransform.get());
                  final Pose3d endPose =
                      AlliancePoseMirror.mirrorPose3d(new Pose3d(blueSpeaker, startPose.getRotation()));

                  final double duration =
                      startPose.getTranslation().getDistance(endPose.getTranslation()) / shotSpeed;
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () -> {
                            Logger.recordOutput(
                                "NoteVisualizer",
                                new Pose3d[] {
                                  startPose.interpolate(endPose, timer.get() / duration)
                                });
                          })
                      .until(() -> timer.hasElapsed(duration))
                      .finallyDo(
                          () -> {
                            Logger.recordOutput("NoteVisualizer", new Pose3d[] {});
                          });
                },
                Set.of())
            .ignoringDisable(true));
  }
}