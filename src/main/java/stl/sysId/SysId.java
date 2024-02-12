package stl.sysId;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Volts;

public class SysId {
    
  public enum CharacterizationRoutine {
    QUASISTATIC_FORWARD,
    QUASISTATIC_BACKWARD,
    DYNAMIC_FORWARD,
    DYNAMIC_BACKWARD,
  } 

  public Command getCharacterizationRoutine(CharacterizableSubsystem subsystem, CharacterizationRoutine routine) {
    var sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
          null, null, null, // Use default config
          (state) -> Logger.recordOutput("SysIdTestState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
          (Measure<Voltage> voltage) -> subsystem.runVolts(voltage.in(Volts)),
          null, // No log consumer, since data is recorded by AdvantageKit
          subsystem
        )
      );
      switch(routine) {
        case QUASISTATIC_FORWARD:
            return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
        case QUASISTATIC_BACKWARD:
            return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse);
        case DYNAMIC_FORWARD:
            return sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward);
        case DYNAMIC_BACKWARD:
            return sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse);
        default:
            return new WaitCommand(1);
      }
  }
}
