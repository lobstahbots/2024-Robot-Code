// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IndexerConstants.IndexerState;
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
  /** Creates a new IntakeNoteCommand. */
  public Intake intake;
  public Indexer indexer;

  public IntakeNoteCommand(Indexer indexer, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.indexer = indexer;
    indexer.setIndexerState(IndexerState.NoNote);
    addRequirements(indexer, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.setIndexerState(IndexerState.NoNote);
    intake.setIntakeMotorSpeed(IntakeConstants.INTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateIndexerState();
    switch(indexer.getIndexerState()) {
      case NoNote:
        intake.setIntakeMotorSpeed(IntakeConstants.INTAKE_SPEED);
        indexer.setIndexerMotorSpeed(IndexerConstants.FAST_INDEXER_MOTOR_SPEED);
        break;
      
      case MovingInIndexer: 
        intake.setIntakeMotorSpeed(IntakeConstants.INTAKE_SPEED);
        indexer.setIndexerMotorSpeed(IndexerConstants.FAST_INDEXER_MOTOR_SPEED);
        break;
      
      case InShooter:
        intake.stopIntakeMotor();
        indexer.setIndexerMotorSpeed(IndexerConstants.SLOW_INDEXER_MOTOR_SPEED);
        break;
      
      default:
        intake.stopIntakeMotor();
        indexer.stopIndexerMotor();
        break;
    }

    SmartDashboard.putBoolean("Flywheel Beam Broken", indexer.flywheelBeamBroken());
    SmartDashboard.putBoolean("Intake Beam Broken", indexer.intakeBeamBroken());
    SmartDashboard.putString("State", indexer.getIndexerState().toString());
  }

  private void updateIndexerState() {
        // if (indexer.intakeBeamBroken() && indexer.flywheelBeamBroken()) indexerState = IndexerState.InIndexer;
        // else if (indexer.intakeBeamBroken()) indexerState = IndexerState.MovingInIndexer;
        // else if (indexer.flywheelBeamBroken()) indexerState = IndexerState.InShooter;
        switch (indexer.getIndexerState()) {
          case NoNote:
            if(indexer.intakeBeamBroken()) indexer.setIndexerState(IndexerState.MovingInIndexer);
              break;
            
          case MovingInIndexer:
            if(!indexer.intakeBeamBroken()) indexer.setIndexerState(IndexerState.InShooter);
            break;
            
          case InShooter:
            if(indexer.intakeBeamBroken()) indexer.setIndexerState(IndexerState.InIndexer);
            break;
          
          case InIndexer:
            break;

          default: 
            indexer.setIndexerState(IndexerState.NoNote);
            break;
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Flywheel Beam Broken", indexer.flywheelBeamBroken());
    SmartDashboard.putBoolean("Intake Beam Broken", indexer.intakeBeamBroken());
    SmartDashboard.putString("State", indexer.getIndexerState().toString());
    indexer.stopIndexerMotor();
    intake.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.getIndexerState() == IndexerState.InIndexer;
  }

}
