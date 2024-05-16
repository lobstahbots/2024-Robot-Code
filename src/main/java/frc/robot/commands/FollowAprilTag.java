// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveBase;

public class FollowAprilTag extends Command {
    private final DriveBase driveBase;
    private final int fidID;
    private final PIDController distancePID = new PIDController(0.1, 0, 0);
    private final PIDController anglePID = new PIDController(0.1, 0, 0);

/*
 * Get Robots inital pos in relation to tag
 * if (tag angle changes) {
 *  rotate to be facing tag
 * }
 * 
 * if (dist to tag changes) {
 *  get to the "right dist"
 * }
*/

  /** Creates a new followAprilTag. */
  public FollowAprilTag(DriveBase driveBase, int fidID) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveBase = driveBase;
    this.fidID = fidID;

    addRequirements(driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(driveBase.getDriveVision(fidID).isPresent()) {
        Transform3d transform3d = driveBase.getDriveVision(fidID).get();
        transformd3d.getRotation
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
