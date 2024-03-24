
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexerConstants.IndexerState;
import frc.robot.subsystems.indexer.Indexer;

public class SpinIndexerCommand extends Command {
  private final Indexer indexer;
  private final double speed;
  /**
   * Command to move the {@link Indexer}.
   * @param indexer The indexer subsystem.
   * @param speed The speed at which to move.
   */
  public SpinIndexerCommand(Indexer indexer, double speed) {
    this.indexer = indexer;
    this.speed = speed;
    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    indexer.setIndexerState(IndexerState.NoNote);
  }

  @Override
  public void execute() {
    indexer.setIndexerMotorSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexerMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
