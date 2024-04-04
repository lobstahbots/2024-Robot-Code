// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;

/**
 * Command to intake a note. It goes through the following steps: <ol>
 *  <li>Spin the intake until the beam break sensor is broken</li>
 *  <li>Spin the indexer until the beam is no longer broken</li>
 *  <li>Spin the indexer backwards until the beam is broken again</li>
 * </ol>
 * After this, the command is done.
 */
public class IntakeNoteCommand extends Command {
//   /** Creates a new IntakeNoteCommand. */
  public Intake intake;
  public Indexer indexer;

  public IntakeNoteCommand(Indexer indexer, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.indexer = indexer;
    addRequirements(indexer, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeMotorSpeed(IntakeConstants.INTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (indexer.intakeBeamBroken() && indexer.flywheelBeamBroken()) {
        indexer.stopIndexerMotor();
        intake.stopIntakeMotor();
    } else if (indexer.intakeBeamBroken()) {
        indexer.setIndexerMotorSpeed(IndexerConstants.FAST_INDEXER_MOTOR_SPEED);
        intake.setIntakeMotorSpeed(IntakeConstants.INTAKE_SPEED);
    } else if (indexer.flywheelBeamBroken()) {
        indexer.setIndexerMotorSpeed(IndexerConstants.SLOW_INDEXER_MOTOR_OUTTAKE_SPEED);
        intake.stopIntakeMotor();
    } else {
        indexer.setIndexerMotorSpeed(IndexerConstants.FAST_INDEXER_MOTOR_SPEED);
        intake.setIntakeMotorSpeed(IntakeConstants.INTAKE_SPEED);
    } 
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.intakeBeamBroken() && indexer.flywheelBeamBroken();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeMotor();
    indexer.stopIndexerMotor();
  }

}
