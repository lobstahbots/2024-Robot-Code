package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.Comparator;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.RotatePivotCommand;
import frc.robot.commands.SpinIndexerCommand;
import frc.robot.commands.SpinIntakeCommand;
import frc.robot.commands.SpinShooterCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveDriveStopCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.TurnToPointCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotKinematics;
import frc.robot.subsystems.shooter.NoteVisualizer;
import frc.robot.subsystems.shooter.Shooter;
import stl.command.PeriodicConditionalCommand;
import stl.sysId.CharacterizableSubsystem;
import stl.trajectory.AlliancePoseMirror;

public class AutoFactory {
    private final Supplier<List<Object>> responses;
    private final DriveBase driveBase;
    private final Intake intake;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Pivot pivot;

    public AutoFactory(DriveBase driveBase, Shooter shooter, Intake intake, Pivot pivot, Indexer indexer,
            Supplier<List<Object>> responsesSupplier) {
        this.responses = responsesSupplier;
        this.driveBase = driveBase;
        this.intake = intake;
        this.shooter = shooter;
        this.pivot = pivot;
        this.indexer = indexer;

        AutoBuilder.configureHolonomic(driveBase::getPose, // Robot pose supplier
                driveBase::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                driveBase::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                driveBase::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                        new PIDConstants(2.25, 0.0, 0), // Translation PID constants
                        new PIDConstants(0.5, 0.0, 0), // Rotation PID constants
                        0.1, // Max module speed, in m/s
                        Units.inchesToMeters(Math.sqrt(35 * 35 + 35 * 35)), // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, false) // Default path replanning config. See the API for the options
                ), () -> {
                    return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                }, driveBase);

        NoteVisualizer.setRobotPoseSupplier(driveBase::getPose, pivot::getPosition);

    }

    /**
     * Determines type of an inputted trajectory - constructed using Choreo, or
     * using PathPlanner. Default should be Choreo.
     */
    public enum PathType {
        CHOREO, PATHPLANNER
    }

    /**
     * Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose The desired end pose of the generated path.
     * @return The constructed path following command
     */
    public Command getPathFindToPoseCommand(Pose2d targetPose) {

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(targetPose, PathConstants.CONSTRAINTS, 0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel
                    // before attempting to rotate.
        ).andThen(new SwerveDriveStopCommand(driveBase));

        return pathfindingCommand;
    }

    /**
     * Constructs a path following command to generate a path to a target position.
     * 
     * @param targetPose Supplier for the desired end pose of the generated path.
     * @return The constructed path following command
     */
    public Command getPathFindToPoseCommand(Supplier<Pose2d> targetPose) {

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPoseFlipped(targetPose.get(), PathConstants.CONSTRAINTS, 0.0, // Goal end velocity in meters/sec
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
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(path, PathConstants.CONSTRAINTS, 0.0 // Rotation delay distance in meters. This is how far the robot should travel
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
        PathPlannerPath path = new PathPlannerPath(bezierPoints, PathConstants.CONSTRAINTS,
                new GoalEndState(0.0, goalEndRotationHolonomic) // Goal end state. You can set a holonomic rotation
                                                                                                                                                    // here. If using a differential drivetrain, the
                                                                                                                                                    // rotation will have no effect.
        );

        Supplier<Command> pathCommand = () -> AutoBuilder.pathfindThenFollowPath(path, PathConstants.CONSTRAINTS, 0.0 // Rotation delay distance in meters. This is how far the robot should travel
                                                                                                                      // before attempting to rotate.
        );
        return pathCommand;
    }

    /* Aim and end. */
    public Command aimOnce(Supplier<Rotation2d> value) {
        return new RotatePivotCommand(pivot, value.get().getDegrees()).until(
                () -> Math.abs(pivot.getPosition().minus(value.get()).getDegrees()) < PivotConstants.MAX_PIVOT_ERROR);
    }

    /* Automatically aim at speaker and stop once it reaches the angle. */
    public Command autoAimOnce() {
        return aimOnce(() -> Rotation2d.fromDegrees(PivotKinematics
                .getShotAngle(() -> FieldConstants.BLUE_ALLIANCE_SPEAKER_POSE3D.toPose2d(), driveBase::getPose)
                .getAsDouble()));
    }

    /* Automatically aim at speaker and hold. */
    public Command autoAimHold() {
        return new RotatePivotCommand(pivot,
                () -> PivotKinematics
                        .getShotAngle(() -> FieldConstants.BLUE_ALLIANCE_SPEAKER_POSE3D.toPose2d(), driveBase::getPose)
                        .getAsDouble());
    }

    /* Automatically aim and shoot note at speaker. */
    public Command aimAndShoot() {
        return autoAimOnce()
                .andThen(new SpinShooterCommand(shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED, false)
                        .alongWith(new WaitCommand(1)
                                .andThen(new SpinIndexerCommand(indexer, IndexerConstants.FAST_INDEXER_MOTOR_SPEED)))
                        .alongWith(autoAimHold()))
                .withTimeout(3);
    }

    /* Hardcoded two-note auto. (BSU) */
    public Command getTwoNote() {
        return aimOnce(() -> Rotation2d.fromDegrees(40))
                .andThen(new SpinShooterCommand(shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED, false)
                        .alongWith(new WaitCommand(2)
                                .andThen(new SpinIndexerCommand(indexer, IndexerConstants.FAST_INDEXER_MOTOR_SPEED)))
                        .alongWith(new RotatePivotCommand(pivot, 40)))
                .withTimeout(5)
                .andThen(aimOnce(() -> new Rotation2d(0))
                        .alongWith(new InstantCommand(() -> shooter.setIdleMode(NeutralModeValue.Brake))))
                .andThen(new SwerveDriveCommand(driveBase, 0.15, 0, 0, true).withTimeout(1.5)
                        .raceWith(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_SPEED)
                                .alongWith(new SpinIndexerCommand(indexer, IndexerConstants.FAST_INDEXER_MOTOR_SPEED))))
                .andThen(aimOnce(() -> Rotation2d.fromDegrees(22)).andThen(
                        new SpinShooterCommand(shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED, false)
                                .alongWith(new WaitCommand(2).andThen(
                                        new SpinIndexerCommand(indexer, IndexerConstants.FAST_INDEXER_MOTOR_SPEED)))
                                .alongWith(new RotatePivotCommand(pivot, 22)))
                        .withTimeout(5));
    }

    public Command getInitialPose() {
        int startingIndex = (int) responses.get().get(0);
        if (startingIndex == 0) {
            return new InstantCommand(() -> driveBase
                    .resetPose(AlliancePoseMirror.mirrorPose2d(new Pose2d(0.61, 6.47, driveBase.getGyroAngle()))));
        } else if (startingIndex == 1) {
            return new InstantCommand(() -> driveBase
                    .resetPose(AlliancePoseMirror.mirrorPose2d(new Pose2d(1.07, 5.46, driveBase.getGyroAngle()))));
        } else {
            return new InstantCommand(() -> driveBase
                    .resetPose(AlliancePoseMirror.mirrorPose2d(new Pose2d(0.51, 4.47, driveBase.getGyroAngle()))));
        }
    }

    /* Hardcoded drive-back auto. (BSU) */
    public Command getDriveAuto() {
        return new SwerveDriveCommand(driveBase, 0.5, 0, 0, false).withTimeout(3);
    }

    /* Hardcoded one-note auto. (BSU) */
    public Command getScoreAuto() {
        return aimOnce(() -> Rotation2d.fromDegrees(40))
                .andThen(new SpinShooterCommand(shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED, false)
                        .alongWith(new WaitCommand(2)
                                .andThen(new SpinIndexerCommand(indexer, IndexerConstants.FAST_INDEXER_MOTOR_SPEED)))
                        .alongWith(new RotatePivotCommand(pivot, 40)))
                .withTimeout(5);
    }

    /* Hardcoded one note and drive back auto. (BSU) */
    public Command getScoreAndDriveAuto() {
        return getScoreAuto().andThen(new SwerveDriveCommand(driveBase, 0.5, 0, 0, false).withTimeout(4));
    }

    public Command getScoreAndDriveAutoAmpSide() {
        return getScoreAuto().andThen(new WaitCommand(5)).andThen(new SwerveDriveCommand(driveBase, 0.15, 0, 0, true).withTimeout(6));
    }

    public Command intake() {
        System.out.println("Scheduled Intaking");
        return new SpinIntakeCommand(intake, IntakeConstants.INTAKE_SPEED)
        .alongWith(new PeriodicConditionalCommand(new SpinIndexerCommand(indexer, 0),
                new SpinIndexerCommand(indexer,
                        IndexerConstants.FAST_INDEXER_MOTOR_SPEED),
                () -> indexer.flywheelBeamBroken() && indexer.intakeBeamBroken()));
    }

    public Command shootTurnIntake() {
        System.out.println("Schedule Shoot Turn");
        return aimAndShoot().andThen(new TurnToAngleCommand(driveBase, new Rotation2d(0), 0, 0, true)).andThen(intake());
    }

    /* Pickup and score one note. */
    public Command pickupAndScore(Pose2d notePoseBlue, Pose2d scoringPose) {
        Pose2d targetPose = FieldConstants.BLUE_ALLIANCE_SPEAKER_POSE3D.toPose2d();
        Logger.recordOutput(notePoseBlue.toString(),
                new Pose2d(notePoseBlue.getX() - FieldConstants.PICKUP_OFFSET, notePoseBlue.getY(), new Rotation2d()));
        Command pickupAndScoreCommand = getPathFindToPoseCommand(
                new Pose2d(notePoseBlue.getX() - FieldConstants.PICKUP_OFFSET, notePoseBlue.getY(), new Rotation2d()))
                        .andThen(new InstantCommand(() -> Logger.recordOutput("Auto Step", 0)))
                        .raceWith(new RotatePivotCommand(pivot, 0))
                        .andThen(new InstantCommand(() -> Logger.recordOutput("Auto Step", 1)))
                        .andThen(new InstantCommand(() -> Logger.recordOutput("Auto Step", 2)))
                        .andThen(new TurnToAngleCommand(driveBase, new Rotation2d(0), 0, 0, true))
                        .andThen(new InstantCommand(() -> Logger.recordOutput("Auto Step", 3)))
                        .andThen(new InstantCommand(() -> shooter.setIdleMode(NeutralModeValue.Brake)))
                        .andThen(new InstantCommand(() -> Logger.recordOutput("Auto Step", 4)))
                        .andThen(new SwerveDriveCommand(driveBase, 0.2, 0, 0, true).withTimeout(0.7)
                                .deadlineWith(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_SPEED)
                                        .alongWith(new PeriodicConditionalCommand(new SpinIndexerCommand(indexer, 0),
                                                new SpinIndexerCommand(indexer,
                                                        IndexerConstants.FAST_INDEXER_MOTOR_SPEED),
                                                () -> indexer.flywheelBeamBroken() && indexer.intakeBeamBroken()))))
                        .andThen(new InstantCommand(() -> Logger.recordOutput("Auto Step", 5)))
                        .andThen(getPathFindToPoseCommand(scoringPose))
                        .andThen(new InstantCommand(() -> Logger.recordOutput("Auto Step", 6)))
                        .andThen(new TurnToPointCommand(driveBase, driveBase::getPose, targetPose, 0, 0, false, true))
                        // .andThen(getScoreAuto())
                        .andThen(new InstantCommand(() -> Logger.recordOutput("Auto Step", 7))).andThen(aimAndShoot());
        return pickupAndScoreCommand;
    }

    @AutoLogOutput
    public BooleanSupplier isWithinTarget(Pose2d target, double zone) {
        return () -> Math.abs(driveBase.getPose().getTranslation()
                .getDistance(AlliancePoseMirror.mirrorTranslation2d(target.getTranslation()))) < zone;
    }

    public Command getWingAndMidlineAuto() {
        int startingWingNoteIndex = (int) responses.get().get(0);
        int endingWingNoteIndex = (int) responses.get().get(1);
        int startingCenterNoteIndex = (int) responses.get().get(2);
        int endingCenterNoteIndex = (int) responses.get().get(3);

        Command autoCommand = aimAndShoot();

        if (startingWingNoteIndex < endingWingNoteIndex) {
            for (int i = startingWingNoteIndex; i <= endingWingNoteIndex; i++) {
                autoCommand = autoCommand.andThen(pickupAndScore(FieldConstants.BLUE_WING_NOTES_STARTING_POSES[i],
                        FieldConstants.SUBWOOFER_SHOOTING_POSE));
            }
        } else {
            for (int i = startingWingNoteIndex; i >= endingWingNoteIndex; i--) {
                autoCommand = autoCommand.andThen(pickupAndScore(FieldConstants.BLUE_WING_NOTES_STARTING_POSES[i],
                        FieldConstants.SUBWOOFER_SHOOTING_POSE));
            }
        }

        if (startingCenterNoteIndex < endingCenterNoteIndex) {
            for (int i = startingCenterNoteIndex; i <= endingCenterNoteIndex; i++) {
                autoCommand = autoCommand.andThen(pickupAndScore(FieldConstants.MIDLINE_NOTES_STARTING_POSES[i],
                        FieldConstants.SHOOTING_POSES[i / 3]));
            }
        } else {
            for (int i = startingCenterNoteIndex; i >= endingCenterNoteIndex; i--) {
                autoCommand = autoCommand.andThen(pickupAndScore(FieldConstants.MIDLINE_NOTES_STARTING_POSES[i],
                        FieldConstants.SHOOTING_POSES[i / 3]));
            }
        }

        return autoCommand;

    }

    public Command autoPickupNote() {
        Pose2d[] notePositions = driveBase.getNotePositions();
        Pose2d pose = driveBase.getPose();
        var selectedNoteOptional = Stream.of(notePositions)
                .sorted(Comparator.comparingDouble(notePose -> notePose.minus(pose).getTranslation().getNorm()))
                .findFirst();
        if (selectedNoteOptional.isEmpty()) return new InstantCommand();

        var noteOffset = selectedNoteOptional.get().minus(driveBase.getPose());
        noteOffset = noteOffset.times(1 + FieldConstants.NOTE_AUTO_PICKUP_OVERSHOOT / noteOffset.getTranslation().getNorm());
        var notePickupPos = driveBase.getPose().plus(noteOffset);
        return new TurnToPointCommand(driveBase, driveBase::getPose, notePickupPos, 0, 0, true, true)
                .andThen(new IntakeNoteCommand(indexer, intake).deadlineWith(getPathFindToPoseCommand(notePickupPos)));
    }

    public enum CharacterizationRoutine {
        QUASISTATIC_FORWARD, QUASISTATIC_BACKWARD, DYNAMIC_FORWARD, DYNAMIC_BACKWARD,
    }

    public Command getCharacterizationRoutine() {
        CharacterizableSubsystem subsystem = (CharacterizableSubsystem) responses.get().get(0);
        CharacterizationRoutine routine = (CharacterizationRoutine) responses.get().get(1);

        var sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, null, null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism((Measure<Voltage> voltage) -> subsystem.runVolts(voltage.in(Volts)), null, // No log consumer, since data is recorded by AdvantageKit
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
