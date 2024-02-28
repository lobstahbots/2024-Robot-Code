// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class SpinShooterCommand extends Command {
  private final Shooter shooter;
  private final double lowerShooterSpeed;
  private final double upperShooterSpeed;

  public SpinShooterCommand(Shooter shooter, double lowerShooterSpeed, double upperShooterSpeed) {
    this.shooter = shooter;
    this.lowerShooterSpeed = lowerShooterSpeed;
    this.upperShooterSpeed = upperShooterSpeed;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    System.out.println("Shooting" + upperShooterSpeed + " " + lowerShooterSpeed);
    shooter.setShooterSpeed(upperShooterSpeed, lowerShooterSpeed);
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
