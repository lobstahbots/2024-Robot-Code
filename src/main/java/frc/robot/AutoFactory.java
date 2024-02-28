package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.RotatePivotCommand;
import frc.robot.commands.SpinIntakeCommand;
import frc.robot.commands.SpinShooterCommand;
import frc.robot.commands.SwerveDriveStopCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.TurnToPointCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotKinematics;
import frc.robot.subsystems.shooter.NoteVisualizer;
import frc.robot.subsystems.shooter.Shooter;
import stl.sysId.CharacterizableSubsystem;
import stl.trajectory.AlliancePoseMirror;

public class AutoFactory {
    private final Supplier<List<Object>> responses;
    private final DriveBase driveBase;
    private final Intake intake;
    private final Shooter shooter;
    private final Pivot pivot;

    public AutoFactory(DriveBase driveBase, Shooter shooter, Intake intake, Pivot pivot,
            Supplier<List<Object>> responsesSupplier) {
        this.responses = responsesSupplier;
        this.driveBase = driveBase;
        this.intake = intake;
        this.shooter = shooter;
        this.pivot = pivot;

        AutoBuilder.configureHolonomic(
                driveBase::getPose, // Robot pose supplier
                driveBase::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                driveBase::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                driveBase::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(5, 0.0, 0.1), // Translation PID constants
                        new PIDConstants(5, 0.0, 0.1), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, false) // Default path replanning config. See the API for the options
                                                         // here
                ),
                () -> {
                    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                },
                driveBase);

        NoteVisualizer.setRobotPoseSupplier(driveBase::getPose, pivot::getPosition);

    }

    /**
     * Determines type of an inputted trajectory - constructed using Choreo, or
     * using PathPlanner. Default should be Choreo.
     */
    public enum PathType {
        CHOREO,
        PATHPLANNER
    }

    /**
     * Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose The desired end pose of the generated path.
     * @return The constructed path following command
     */
    public Command getPathFindToPoseCommand(Pose2d targetPose) {

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(
                targetPose,
                PathConstants.CONSTRAINTS,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        ).andThen(new SwerveDriveStopCommand(driveBase));

        return pathfindingCommand;
    }

    /**
     * Constructs a path following command to a preset path from the deploy
     * directory Path can be PathPlanner or Choreo-constructed.
     * 
     * @param pathname A String containing the name of the file with the path (leave
     *                 out the .traj or .path ending).
     * @param pathType A {@link PathType} determining the format of the inputted
     *                 trajectory. Files ending in .path should be imported as
     *                 PATHPLANNER, while files ending in .traj should be imported
     *                 as CHOREO.
     * @return The constructed path following command
     */
    public Command getPathFindToPathCommand(String pathname, PathType pathType) {
        PathPlannerPath path;
        switch (pathType) {
            case CHOREO:
                path = PathPlannerPath.fromChoreoTrajectory(pathname);
                break;
            case PATHPLANNER:
                path = PathPlannerPath.fromPathFile(pathname);
                break;
            default:
                path = PathPlannerPath.fromChoreoTrajectory(pathname);
        }

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path,
                PathConstants.CONSTRAINTS,
                3.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        );

        return pathfindingCommand;
    }

    /**
     * Constructs a path following command through a provided set of waypoints. Ends
     * with desired holonomic rotation.
     * 
     * @param goalEndRotationHolonomic Desired holonomic end rotation
     * @param poses                    List of bezier poses. Each {@link Pose2d}
     *                                 represents one waypoint. The rotation
     *                                 component of the pose should be the direction
     *                                 of travel. Do not use holonomic rotation.
     * @return The constructed path following command through provided poses, with
     *         set end rotation.
     */
    public Supplier<Command> getPathFromWaypoints(Rotation2d goalEndRotationHolonomic, Pose2d... poses) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(poses);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints, PathConstants.CONSTRAINTS,
                new GoalEndState(0.0, goalEndRotationHolonomic) // Goal end state. You can set a holonomic rotation
                                                                // here. If using a differential drivetrain, the
                                                                // rotation will have no effect.
        );

        Supplier<Command> pathCommand = () -> AutoBuilder.pathfindThenFollowPath(
                path,
                PathConstants.CONSTRAINTS,
                3.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        );
        return pathCommand;
    }

    public Command getSimpleAuto() {
        int driverStation = (int) responses.get().get(0);
        return getPathFindToPoseCommand(FieldConstants.BLUE_WING_NOTES_STARTING_POSES[driverStation - 1]);
    }

    public Command getPivotCommand(Rotation2d value) {
        return new RotatePivotCommand(pivot, value.getRadians())
                .until(() -> pivot.getPosition().minus(value).getDegrees() < PivotConstants.MAX_PIVOT_ERROR);
    }

    public Command getShootCommand() {
        return new WaitCommand(ShooterConstants.SHOOT_TIME).deadlineWith(
                new SpinShooterCommand(shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED));
    }

    public Command getOneNoteAuto() {
        return getPathFindToPoseCommand(AutoConstants.FIRST_NOTE_SHOOTING_POSITION)
                .alongWith(getPivotCommand(AutoConstants.FIRST_NOTE_SHOOTING_ANGLE))
                .andThen(getShootCommand());
    }

    public Command getTwoNoteAuto() {
        return getOneNoteAuto().andThen(new PathPlannerAuto("2 Note Auto"));
    }

    public Command getThreeNoteAuto() {
        return getOneNoteAuto().andThen(new PathPlannerAuto("3 Note Auto"));
    }

    public Command getFourNoteAuto() {
        return getOneNoteAuto().andThen(new PathPlannerAuto("4 Note Auto"));
    }

    public Command pickupAndScore(Pose2d notePoseBlue) {
        Pose2d targetPose = AlliancePoseMirror.mirrorPose2d(FieldConstants.BLUE_ALLIANCE_SPEAKER_POSE3D.toPose2d());
        Command pickupAndScoreCommand = getPathFindToPoseCommand(
                AlliancePoseMirror.mirrorPose2d(notePoseBlue
                        .plus(new Transform2d(-FieldConstants.PICKUP_OFFSET, 0, new Rotation2d()))))
                .andThen(new SwerveDriveStopCommand(driveBase))
                .andThen(new TurnToAngleCommand(driveBase, new Rotation2d(0)))
                .andThen(getPathFindToPoseCommand(AlliancePoseMirror
                        .mirrorPose2d(notePoseBlue))
                        .raceWith(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_SPEED))
                        .andThen(getPivotCommand(new Rotation2d(PivotKinematics.getShotAngle(() -> targetPose, driveBase::getPose).getAsDouble()))
                                .raceWith(new TurnToPointCommand(driveBase::getPose, targetPose, driveBase)))
                        .andThen(new SpinShooterCommand(shooter, -ShooterConstants.SHOOTER_SPEED,
                                ShooterConstants.SHOOTER_SPEED).withTimeout(1)));
        return pickupAndScoreCommand;
    }

    public Command getWingAndMidlineAuto() {
        int startingWingNoteIndex = (int) responses.get().get(0);
        int endingWingNoteIndex = (int) responses.get().get(1);
        int startingCenterNoteIndex = (int) responses.get().get(2);
        int endingCenterNoteIndex = (int) responses.get().get(3);

        Command autoCommand = new WaitCommand(0);

        if (startingWingNoteIndex < endingWingNoteIndex) {
            for (int i = startingWingNoteIndex; i <= endingWingNoteIndex; i++) {
                autoCommand = autoCommand
                        .andThen(pickupAndScore(FieldConstants.BLUE_WING_NOTES_STARTING_POSES[i]));
            }
        } else {
            for (int i = startingWingNoteIndex; i >= endingWingNoteIndex; i--) {
                autoCommand = autoCommand
                        .andThen(pickupAndScore(FieldConstants.BLUE_WING_NOTES_STARTING_POSES[i]));
            }
        }

        if (startingCenterNoteIndex < endingCenterNoteIndex) {
            for (int i = startingCenterNoteIndex; i <= endingCenterNoteIndex; i++) {
                autoCommand = autoCommand.andThen(pickupAndScore(FieldConstants.MIDLINE_NOTES_STARTING_POSES[i]));
            }
        } else {
            for (int i = startingCenterNoteIndex; i >= endingCenterNoteIndex; i--) {
                autoCommand = autoCommand.andThen(pickupAndScore(FieldConstants.MIDLINE_NOTES_STARTING_POSES[i]));
            }
        }

        return autoCommand;

    }

    public enum CharacterizationRoutine {
        QUASISTATIC_FORWARD,
        QUASISTATIC_BACKWARD,
        DYNAMIC_FORWARD,
        DYNAMIC_BACKWARD,
    }

    public Command getCharacterizationRoutine() {
        CharacterizableSubsystem subsystem = (CharacterizableSubsystem) responses.get().get(0);
        CharacterizationRoutine routine = (CharacterizationRoutine) responses.get().get(1);

        var sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, // Use default config
                        (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> voltage) -> subsystem.runVolts(voltage.in(Volts)),
                        null, // No log consumer, since data is recorded by AdvantageKit
                        subsystem));
        switch (routine) {
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
