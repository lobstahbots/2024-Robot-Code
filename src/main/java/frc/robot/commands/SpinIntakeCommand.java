
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeReal;

public class SpinIntakeCommand extends Command {
  private final IntakeReal intake;
  private final double speed;
  /**
   * Command to move the intake ({@link IntakeReal})
   * @param intake The intake subsystem.
   * @param speed The speed at which to move.
   */
  public SpinIntakeCommand(IntakeReal intake, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIMotorSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
