// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSparkMax;

public class StopShooterCommand extends Command {
  private final ShooterSparkMax shooter;
  
  public StopShooterCommand(ShooterSparkMax shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
