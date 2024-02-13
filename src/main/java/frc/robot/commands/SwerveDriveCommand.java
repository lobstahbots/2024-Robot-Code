// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.drive.DriveBase;

/** A command to drive a swerve robot. */
public class SwerveDriveCommand extends Command {
  private final DriveBase driveBase;
  private final DoubleSupplier strafeXSupplier;
  private final DoubleSupplier strafeYSupplier;
  private final DoubleSupplier rotationSupplier;
  private final boolean fieldCentric;

  public SwerveDriveCommand(DriveBase driveBase, DoubleSupplier strafeXSupplier, DoubleSupplier strafeYSupplier, DoubleSupplier rotationSupplier, boolean fieldCentric) {
    this.driveBase = driveBase;
    this.strafeXSupplier = strafeXSupplier;
    this.strafeYSupplier = strafeYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldCentric = fieldCentric;
    addRequirements(driveBase);
  }

  public SwerveDriveCommand(DriveBase driveBase, double strafeX, double strafeY, double rotation, boolean fieldCentric) {
    this(driveBase, () -> strafeX, () -> strafeY, () -> rotation, fieldCentric);
  }

  @Override
  public void execute() {
     double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble()), IOConstants.JOYSTICK_DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), IOConstants.JOYSTICK_DEADBAND);

          // Square values
          // linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calculate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();
            
    driveBase.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(linearVelocity.getX() * DriveConstants.MAX_DRIVE_SPEED, linearVelocity.getY() * DriveConstants.MAX_DRIVE_SPEED, omega * DriveConstants.MAX_ANGULAR_SPEED, driveBase.getGyroAngle()));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
