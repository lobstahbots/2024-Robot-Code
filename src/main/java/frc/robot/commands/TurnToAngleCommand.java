// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.drive.DriveBase;
import stl.math.LobstahMath;

public class TurnToAngleCommand extends Command {
  public final DriveBase driveBase;
  public final PIDController pidController = new PIDController(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD);
  public final Supplier<Rotation2d> desiredRotation;
  private final DoubleSupplier strafeXSupplier;
  private final DoubleSupplier strafeYSupplier;
  private final BooleanSupplier fieldCentric;
  private final boolean end;

  /**
   * Creates a TurnToAngleCommand which turns the robot to a specific angle
   * @param driveBase The subsystem to control
   * @param desiredRotation The desired angle to rotate to
   * @param strafeXSupplier A supplier for the X component of the robot translation.
   * @param strafeYSupplier A supplier for the Y component of the robot translation.
   * @param fieldCentric Whether the robot drives field centric. Does not affect rotation.
   */
  public TurnToAngleCommand(DriveBase driveBase, Supplier<Rotation2d> desiredRotation, DoubleSupplier strafeXSupplier, DoubleSupplier strafeYSupplier, BooleanSupplier fieldCentric, boolean end) {
    this.driveBase = driveBase;
    this.strafeXSupplier = strafeXSupplier;
    this.strafeYSupplier = strafeYSupplier;
    this.desiredRotation = desiredRotation;
    this.fieldCentric = fieldCentric;
    this.end = end;
    addRequirements(driveBase);
  }

  public TurnToAngleCommand(DriveBase driveBase, Supplier<Rotation2d> desiredRotation, DoubleSupplier strafeXSupplier, DoubleSupplier strafeYSupplier, BooleanSupplier fieldCentric) {
    this(driveBase, desiredRotation, strafeXSupplier, strafeYSupplier, fieldCentric, false);
  }

  /**
   * Creates a TurnToAngleCommand which turns the robot to a specific angle
   * @param driveBase The subsystem to control
   * @param desiredRotation The desired angle to rotate to
   * @param strafeXSupplier The X component of the robot translation.
   * @param strafeYSupplier The Y component of the robot translation.
   * @param fieldCentric Whether the robot drives field centric. Does not affect rotation.
   */
  public TurnToAngleCommand(DriveBase driveBase, Rotation2d desiredRotation, double strafeX, double strafeY, boolean fieldCentric) {
    this(driveBase, () -> desiredRotation, () -> strafeX, () -> strafeY, () -> fieldCentric, true);
  }

  /*
   * Gets the error between current gyro angle and the desired angle.
   */
  private double getError() {
    return driveBase.getGyroAngle().getRadians() - desiredRotation.get().getRadians();
  }

  @Override
  public void initialize() {
    pidController.reset();
  }

  @Override
  public void execute() {
    Logger.recordOutput("Desired angle", new Pose2d(driveBase.getPose().getTranslation(), desiredRotation.get()));
    pidController.setSetpoint(desiredRotation.get().getRadians());
    double turnOutput = pidController.calculate(driveBase.getGyroAngle().getRadians(), desiredRotation.get().getRadians());
    if (fieldCentric.getAsBoolean()) {
      double linearMagnitude = MathUtil.applyDeadband(
          Math.hypot(strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble()), IOConstants.JOYSTICK_DEADBAND);
      Rotation2d linearDirection = new Rotation2d(-strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble());

      // Calculate new linear velocity
      Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
          .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
          .getTranslation();

      driveBase.driveRobotRelative(
          ChassisSpeeds.fromFieldRelativeSpeeds(linearVelocity.getX() * DriveConstants.MAX_DRIVE_SPEED,
              linearVelocity.getY() * DriveConstants.MAX_DRIVE_SPEED, turnOutput,
              driveBase.getGyroAngle()));
    } else {
      driveBase.driveRobotRelative(new ChassisSpeeds(strafeXSupplier.getAsDouble(), strafeYSupplier.getAsDouble(), turnOutput));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("end? " + end + "and... " + MathUtil.applyDeadband(getError(), DriveConstants.TURN_DEADBAND));
    return end && MathUtil.applyDeadband(getError(), DriveConstants.TURN_DEADBAND) == 0;
  }
}
