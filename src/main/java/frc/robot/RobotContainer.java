// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AutoFactory.CharacterizationRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.auto.AutonSelector;
import frc.robot.auto.AutonSelector.AutoQuestion;
import frc.robot.commands.RotatePivotCommand;
import frc.robot.commands.SpinIndexerCommand;
import frc.robot.commands.SpinIntakeCommand;
import frc.robot.commands.SpinShooterCommand;
import frc.robot.commands.StopShooterCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.TurnToPointCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.SwerveModuleIOSparkMax;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerSparkMax;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOSparkMax;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOSim;
import stl.command.PeriodicConditionalCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSparkMax;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax.IdleMode;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveBase driveBase;
    private final Pivot pivot;
    private final Shooter shooter;
    private final Indexer indexer = new Indexer(new IndexerSparkMax(IndexerConstants.INDEXER_MOTOR_ID));;
    private final Intake intake = new Intake(
            new IntakeIOSparkMax(IntakeConstants.INTAKE_MOTOR_ID));

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final Joystick driverJoystick = new Joystick(IOConstants.DRIVER_CONTROLLER_PORT);
    private final JoystickButton alignToAmpButton = new JoystickButton(driverJoystick,
            IOConstants.ALIGN_TO_AMP_BUTTON_ID);
    private final JoystickButton alignToSourceButton = new JoystickButton(driverJoystick,
            IOConstants.ALIGN_TO_SOURCE_BUTTON_ID);
    private final JoystickButton alignToSpeakerButton = new JoystickButton(driverJoystick,
            IOConstants.ALIGN_TO_SPEAKER_BUTTON_ID);
    private final Joystick operatorJoystick = new Joystick(IOConstants.OPERATOR_CONTROLLER_PORT);
    private final JoystickButton shooterButton = new JoystickButton(operatorJoystick, IOConstants.SHOOTER_BUTTON_ID);
    private final JoystickButton unshooterButton = new JoystickButton(operatorJoystick,
            IOConstants.UNSHOOTER_BUTTON_ID);
    private final JoystickButton intakeButton = new JoystickButton(driverJoystick, IOConstants.INTAKE_BUTTON_ID);
    private final JoystickButton ampButton = new JoystickButton(operatorJoystick, IOConstants.AMP_BUTTON_ID);
    private final JoystickButton outtakeButton = new JoystickButton(operatorJoystick, IOConstants.OUTTAKE_BUTTON_ID);
    private final JoystickButton indexButton = new JoystickButton(operatorJoystick, IOConstants.INDEXER_BUTTON_ID);
    private final POVButton subwooferButton = new POVButton(operatorJoystick, 0); // UP
    private final POVButton wingButton = new POVButton(operatorJoystick, 90); // RIGHT
    private final POVButton podiumButton = new POVButton(operatorJoystick, 270); // LEFT
  private final JoystickButton sourceButton = new JoystickButton(operatorJoystick, IOConstants.SOURCE_BUTTON_ID); //DOWN
    private final JoystickButton slowdownButton = new JoystickButton(driverJoystick, IOConstants.SLOWDOWN_BUTTON_ID);

    private final AutonSelector<Object> autoChooser = new AutonSelector<>("Auto Chooser", "Do Nothing", List.of(),
            () -> Commands.none());
    private final AutoFactory autoFactory;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Robot.isReal()) {
            SwerveModuleIOSparkMax frontLeft = new SwerveModuleIOSparkMax(FrontLeftModuleConstants.moduleID,
                    "Front left ",
                    FrontLeftModuleConstants.angleID, FrontLeftModuleConstants.driveID,
                    FrontLeftModuleConstants.angleOffset,
                    FrontLeftModuleConstants.inverted);
            SwerveModuleIOSparkMax frontRight = new SwerveModuleIOSparkMax(FrontRightModuleConstants.moduleID,
                    " Front right",
                    FrontRightModuleConstants.angleID, FrontRightModuleConstants.driveID,
                    FrontRightModuleConstants.angleOffset,
                    FrontRightModuleConstants.inverted);
            SwerveModuleIOSparkMax backLeft = new SwerveModuleIOSparkMax(BackLeftModuleConstants.moduleID, " Back left",
                    BackLeftModuleConstants.angleID, BackLeftModuleConstants.driveID,
                    BackLeftModuleConstants.angleOffset,
                    BackLeftModuleConstants.inverted);
            SwerveModuleIOSparkMax backRight = new SwerveModuleIOSparkMax(BackRightModuleConstants.moduleID,
                    "Back right",
                    BackRightModuleConstants.angleID, BackRightModuleConstants.driveID,
                    BackRightModuleConstants.angleOffset,
                    BackRightModuleConstants.inverted);

            driveBase = new DriveBase(new GyroIONavX(), new Vision(new VisionIOPhoton()), frontLeft, frontRight,
                    backLeft, backRight, false);
            pivot = new Pivot(new PivotIOSparkMax(PivotConstants.LEFT_MOTOR_ID, PivotConstants.RIGHT_MOTOR_ID,
                    PivotConstants.ENCODER_CHANNEL));
            shooter = new Shooter(
                    new ShooterIOTalonFX(ShooterConstants.UPPER_SHOOTER_ID, ShooterConstants.LOWER_SHOOTER_ID));
        } else {
            SwerveModuleIOSim frontLeft = new SwerveModuleIOSim(FrontLeftModuleConstants.angleOffset);
            SwerveModuleIOSim frontRight = new SwerveModuleIOSim(FrontRightModuleConstants.angleOffset);
            SwerveModuleIOSim backLeft = new SwerveModuleIOSim(BackLeftModuleConstants.angleOffset);
            SwerveModuleIOSim backRight = new SwerveModuleIOSim(BackRightModuleConstants.angleOffset);

            driveBase = new DriveBase(new GyroIO() {
            }, new Vision(new VisionIOSim()), frontLeft, frontRight, backLeft, backRight, false);
            pivot = new Pivot(new PivotIOSim());
            shooter = new Shooter(
                    new ShooterIOSim());
        }

        this.autoFactory = new AutoFactory(driveBase, shooter, intake, pivot, indexer, autoChooser::getResponses);

        setDefaultCommands();
        smartDashSetup();
        configureButtonBindings();
    }

    private void setDefaultCommands() {
        driveBase.setDefaultCommand(
                new SwerveDriveCommand(driveBase,
                        () -> driverJoystick.getRawAxis(IOConstants.STRAFE_Y_AXIS),
                        () -> driverJoystick.getRawAxis(IOConstants.STRAFE_X_AXIS),
                        () -> driverJoystick.getRawAxis(IOConstants.ROTATION_AXIS),
                        () -> DriveConstants.FIELD_CENTRIC));
        pivot.setDefaultCommand(new RotatePivotCommand(pivot,
                () -> pivot.getPosition().getDegrees() + -5 * MathUtil
                        .applyDeadband(operatorJoystick.getRawAxis(IOConstants.PIVOT_ANGLE_AXIS),
                                PivotConstants.INPUT_DEADBAND)));
        shooter.setDefaultCommand(new PeriodicConditionalCommand(
                new SpinShooterCommand(shooter, -ShooterConstants.SPIN_UP_SPEED, ShooterConstants.SPIN_UP_SPEED),
                new StopShooterCommand(shooter),
                autoFactory.isWithinTarget(FieldConstants.BLUE_ALLIANCE_SPEAKER_POSE3D.toPose2d(),
                        PathConstants.SPIN_UP_FLYWHEELS_RADIUS_METERS)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getCommand();
    }

    public void configureButtonBindings() {
        alignToAmpButton
                .whileTrue(new PeriodicConditionalCommand(
                        new RotatePivotCommand(pivot, PivotConstants.AMP_ANGLE_SETPOINT)
                                .alongWith(new SpinShooterCommand(shooter, -ShooterConstants.AMP_SPEED,
                                        ShooterConstants.AMP_SPEED)),
                        autoFactory.getPathFindToPoseCommand(FieldConstants.BLUE_ALLIANCE_AMP_POSE2D)
                                .andThen(new TurnToAngleCommand(driveBase,
                                        () -> FieldConstants.BLUE_ALLIANCE_AMP_POSE2D.getRotation(),
                                        () -> 0,
                                        () -> 0,
                                        () -> DriveConstants.FIELD_CENTRIC)
                                        .alongWith(new RotatePivotCommand(pivot, PivotConstants.AMP_ANGLE_SETPOINT))
                                        .raceWith(
                                                new SpinShooterCommand(shooter, -ShooterConstants.AMP_SPEED,
                                                        ShooterConstants.AMP_SPEED))),
                        autoFactory.isWithinTarget(FieldConstants.BLUE_ALLIANCE_AMP_POSE2D,
                                PathConstants.AMP_ALIGN_DEADBAND)));
        alignToSourceButton
                .whileTrue(new PeriodicConditionalCommand(
                        new RotatePivotCommand(pivot, PivotConstants.SOURCE_PICKUP_ANGLE_SETPOINT)
                                .alongWith(new SpinShooterCommand(shooter, -ShooterConstants.UNSHOOTER_SPEED,
                                        ShooterConstants.UNSHOOTER_SPEED)),
                        autoFactory.getPathFindToPoseCommand(FieldConstants.BLUE_ALLIANCE_SOURCE_POSE2D)
                                .andThen(new TurnToAngleCommand(driveBase,
                                        () -> FieldConstants.BLUE_ALLIANCE_SOURCE_POSE2D.getRotation(),
                                        () -> 0,
                                        () -> 0,
                                        () -> DriveConstants.FIELD_CENTRIC)
                                        .alongWith(new RotatePivotCommand(pivot,
                                                PivotConstants.SOURCE_PICKUP_ANGLE_SETPOINT))
                                        .raceWith(new SpinShooterCommand(shooter, -ShooterConstants.UNSHOOTER_SPEED,
                                                ShooterConstants.UNSHOOTER_SPEED))),
                        autoFactory.isWithinTarget(FieldConstants.BLUE_ALLIANCE_SOURCE_POSE2D,
                                PathConstants.SOURCE_ALIGN_DEADBAND)));
        alignToSpeakerButton
                .whileTrue(new TurnToPointCommand(driveBase, driveBase::getPose,
                        FieldConstants.BLUE_ALLIANCE_SPEAKER_POSE3D.toPose2d(),
                        () -> driverJoystick.getRawAxis(IOConstants.STRAFE_Y_AXIS),
                        () -> -driverJoystick.getRawAxis(IOConstants.STRAFE_X_AXIS),
                        () -> DriveConstants.FIELD_CENTRIC).alongWith(autoFactory.autoAimHold()));
        slowdownButton.whileTrue(new SwerveDriveCommand(driveBase,
                () -> DriveConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(IOConstants.STRAFE_Y_AXIS),
                () -> -DriveConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(IOConstants.STRAFE_X_AXIS),
                () -> driverJoystick.getRawAxis(IOConstants.ROTATION_AXIS),
                () -> DriveConstants.FIELD_CENTRIC));
        intakeButton.whileTrue(new PeriodicConditionalCommand(
                new SpinIntakeCommand(intake, IntakeConstants.INTAKE_SPEED)
                        .alongWith(new SpinIndexerCommand(indexer, IndexerConstants.FAST_INDEXER_MOTOR_SPEED)),
                new RotatePivotCommand(pivot, 0), () -> Math
                        .abs(pivot.getPosition().getDegrees()) < PivotConstants.MAX_PIVOT_ERROR));
        indexButton.whileTrue(new SpinIndexerCommand(indexer, IndexerConstants.FAST_INDEXER_MOTOR_SPEED)
                .onlyIf(() -> shooter.getLowerFlywheelVelocityRPS() > ShooterConstants.SHOOTING_FLYWHEEL_VELOCITY_RPS
                        && shooter.getUpperFlywheelVelocityRPS() > ShooterConstants.SHOOTING_FLYWHEEL_VELOCITY_RPS));
        shooterButton
                .whileTrue(new SpinShooterCommand(shooter, -ShooterConstants.SHOOTER_SPEED,
                        ShooterConstants.SHOOTER_SPEED));
        unshooterButton.whileTrue(
                new SpinShooterCommand(shooter, -ShooterConstants.UNSHOOTER_SPEED, ShooterConstants.UNSHOOTER_SPEED));
        ampButton.whileTrue(new RotatePivotCommand(pivot, PivotConstants.AMP_ANGLE_SETPOINT)
                .alongWith(new SpinShooterCommand(shooter, -ShooterConstants.AMP_SPEED, ShooterConstants.AMP_SPEED)));
        outtakeButton.whileTrue(new SpinIntakeCommand(intake, IntakeConstants.OUTTAKE_SPEED)
                .alongWith(new SpinIndexerCommand(indexer, IndexerConstants.SLOW_INDEXER_MOTOR_SPEED)));
        subwooferButton.whileTrue(new RotatePivotCommand(pivot, PivotConstants.SUBWOOFER_ANGLE_SETPOINT)
                .alongWith(new SpinShooterCommand(shooter, -ShooterConstants.SHOOTER_SPEED,
                        ShooterConstants.SHOOTER_SPEED)));
        wingButton.whileTrue(new RotatePivotCommand(pivot, 15)
                .alongWith(new SpinShooterCommand(shooter, -ShooterConstants.SHOOTER_SPEED,
                        ShooterConstants.SHOOTER_SPEED)));
        podiumButton.whileTrue(new RotatePivotCommand(pivot, 20)
                .alongWith(new SpinShooterCommand(shooter, -ShooterConstants.SHOOTER_SPEED,
                        ShooterConstants.SHOOTER_SPEED)));
      sourceButton.whileTrue(new RotatePivotCommand(pivot, 108).alongWith(new  SpinShooterCommand(shooter, -ShooterConstants.UNSHOOTER_SPEED, ShooterConstants.UNSHOOTER_SPEED)));
  }

    public boolean getOperatorConnected() {
        return operatorJoystick.isConnected();
    }

    public boolean getDriverConnected() {
        return driverJoystick.isConnected();
    }

    public void smartDashSetup() {

        autoChooser.addRoutine("Characterize", List.of(
                new AutoQuestion<>("Which Subsystem?", Map.of("DriveBase", driveBase, "Pivot", pivot)),
                new AutoQuestion<>("Which Routine",
                        Map.of("Quasistatic Foward", CharacterizationRoutine.QUASISTATIC_FORWARD,
                                "Quasistatic Backward", CharacterizationRoutine.QUASISTATIC_BACKWARD, "Dynamic Forward",
                                CharacterizationRoutine.DYNAMIC_FORWARD, "Dynamic Backward",
                                CharacterizationRoutine.DYNAMIC_BACKWARD))),
                autoFactory::getCharacterizationRoutine);

        autoChooser.addRoutine("Drive", List.of(), autoFactory::getDriveAuto);
        autoChooser.addRoutine("Score Preload", List.of(), autoFactory::getScoreAuto);
        autoChooser.addRoutine("Score Preload And Drive", List.of(), autoFactory::getScoreAndDriveAuto);
        autoChooser.addRoutine("2 Note Subwoofer Center Auto", List.of(), autoFactory::getTwoNote);

        autoChooser.addRoutine("Wing And Midline Auto", List.of(
                new AutoQuestion<>("Starting Note?", Map.of("Wing Right", 0, "Wing Center", 1, "Wing Left", 2)),
                new AutoQuestion<>("Last Wing Note?", Map.of("-Wing Right", 0, "-Wing Center", 1, "-Wing Left", 2)),
                new AutoQuestion<>("Starting Center Line Note?",
                        Map.of("Midline Left", 0, "Midline Center Left", 1, "Midline Center", 2, "Midline Center Right",
                                3,
                                "Midline Right (Source Side)", 4)),
                new AutoQuestion<>("Starting Center Line Note?",
                        Map.of("-Midline Left", 0, "-Midline Center Left", 1, "-Midline Center", 2,
                                "-Midline Center Right", 3,
                                "-Midline Right (Source Side)", 4))),
                autoFactory::getWingAndMidlineAuto);
    }

    public void setIdleMode(IdleMode idleMode, NeutralModeValue shooterIdleMode) {
        driveBase.setIdleMode(idleMode);
        pivot.setIdleMode(idleMode);
        intake.setIdleMode(idleMode);
        shooter.setIdleMode(shooterIdleMode);
    }
}
