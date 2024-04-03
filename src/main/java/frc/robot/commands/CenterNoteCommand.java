// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.indexer.Indexer;

/**
 * Command to intake a note. It goes through the following steps: <ol>
 *  <li>Spin the intake until the beam break sensor is broken</li>
 *  <li>Spin the indexer until the beam is no longer broken</li>
 *  <li>Spin the indexer backwards until the beam is broken again</li>
 * </ol>
 * After this, the command is done.
 */
public class CenterNoteCommand extends Command {
//   /** Creates a new IntakeNoteCommand. */
  public Indexer indexer;

  public CenterNoteCommand(Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.setIndexerMotorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexer.intakeBeamBroken() && indexer.flywheelBeamBroken()) {
        indexer.stopIndexerMotor();
    } else if (indexer.intakeBeamBroken()) {
        indexer.setIndexerMotorSpeed(IndexerConstants.FAST_INDEXER_MOTOR_SPEED);
    } else if (indexer.flywheelBeamBroken()) {
        indexer.setIndexerMotorSpeed(-IndexerConstants.SLOW_INDEXER_MOTOR_SPEED);
    } else {
        indexer.setIndexerMotorSpeed(IndexerConstants.FAST_INDEXER_MOTOR_SPEED);
    } 
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.intakeBeamBroken() && indexer.flywheelBeamBroken();
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexerMotor();
  }

}
