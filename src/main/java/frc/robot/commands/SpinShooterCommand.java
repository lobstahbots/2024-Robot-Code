// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SpinShooterCommand extends Command {
  private final Shooter shooter;
  private final double shooterSpeed;

  public SpinShooterCommand() {
    this.shooter = shooter;
    this.shooterSpeed = shooterSpeed;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.setIMotorSpeed(shooterSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
