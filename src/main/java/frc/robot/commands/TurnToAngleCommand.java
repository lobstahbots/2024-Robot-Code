// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveBase;

public class TurnToAngleCommand extends Command {
  public final DriveBase driveBase;
  public final boolean isBlueAlliance;
  public final PIDController pidController = new PIDController(DriveConstants.TURN_KP, DriveConstants.TURN_KI, DriveConstants.TURN_KD);
  public final Rotation2d desiredRotation;

  /**
   * Creats a TurnToAngleCommand which turns the robot to a specific angle
   * @param driveBase The subsystem to control
   * @param desiredRotation The desired angle to rotate to
   */

  public TurnToAngleCommand(DriveBase driveBase, Rotation2d desiredRotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveBase = driveBase;
    this.isBlueAlliance = DriverStation.getAlliance().get() == Alliance.Blue;
    this.desiredRotation = desiredRotation;
    addRequirements(driveBase);
  }

  private double getError() {
    return driveBase.getGyroAngle().getRadians() - desiredRotation.getRadians();
  }

  @Override
  public void execute() {
    pidController.setSetpoint(desiredRotation.getRadians());
    driveBase.driveRobotRelative(new ChassisSpeeds(0, 0, pidController.calculate(getError(), desiredRotation.getRadians())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveBase.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MathUtil.applyDeadband(getError(), DriveConstants.TURN_DEADBAND) == 0;
  }
}
