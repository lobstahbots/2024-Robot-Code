// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.NoteVisualizer;
import frc.robot.subsystems.shooter.Shooter;

public class SpinShooterCommand extends Command {
  private final Shooter shooter;
  private final double lowerShooterSpeed;
  private final double upperShooterSpeed;
  private final boolean openLoop;

  public SpinShooterCommand(Shooter shooter, double lowerShooterSpeed, double upperShooterSpeed, boolean openLoop) {
    this.shooter = shooter;
    this.lowerShooterSpeed = lowerShooterSpeed;
    this.upperShooterSpeed = upperShooterSpeed;
    this.openLoop = openLoop;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(NoteVisualizer.shoot());
    shooter.setIdleMode(NeutralModeValue.Coast);
  }

  @Override
  public void execute() {
    if (openLoop) shooter.setShooterSpeedRaw(upperShooterSpeed, lowerShooterSpeed); 
    else shooter.setShooterSpeed(upperShooterSpeed, lowerShooterSpeed);
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
