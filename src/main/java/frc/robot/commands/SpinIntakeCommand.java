
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SpinIntakeCommand extends Command {
  private final Intake intake;
  private final double speed;
  /**
   * Command to move the {@link Intake}.
   * @param intake The intake subsystem.
   * @param speed The speed at which to move.
   */
  public SpinIntakeCommand(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.setIntakeMotorSpeed(speed);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
