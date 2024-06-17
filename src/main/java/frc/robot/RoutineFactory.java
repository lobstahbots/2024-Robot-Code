package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoFactory.PathType;

public class RoutineFactory {
    private final AutoFactory autoFactory;
    public RoutineFactory(AutoFactory autoFactory) {
        this.autoFactory = autoFactory;
    }

    public Command getCleanup() {
        System.out.println("Initializing Auto");
        return autoFactory.getPathFindToPathCommand("Cleanup", PathType.CHOREO);
    }
}
