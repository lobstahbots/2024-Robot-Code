// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AutoFactory.CharacterizationRoutine;
import frc.robot.AutoFactory.PathType;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.Constants.IOConstants.DriverIOConstants;
import frc.robot.Constants.IOConstants.OperatorIOConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.auto.AutonSelector;
import frc.robot.auto.AutonSelector.AutoQuestion;
import frc.robot.commands.CenterNoteCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.RotatePivotCommand;
import frc.robot.commands.SpinIndexerCommand;
import frc.robot.commands.SpinIntakeCommand;
import frc.robot.commands.SpinShooterCommand;
import frc.robot.commands.StopIntakeCommand;
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
import stl.trajectory.AlliancePoseMirror;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSparkMax;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsReal;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
    private final LEDs leds = new LEDs(new LEDsReal(LEDConstants.LED_PORT, LEDConstants.LengthConstants.TOTAL));
    private final DriveBase driveBase;
    private final Pivot pivot;
    private final Shooter shooter;
    private final Indexer indexer = new Indexer(new IndexerSparkMax(IndexerConstants.INDEXER_MOTOR_ID));
    private final Intake intake = new Intake(new IntakeIOSparkMax(IntakeConstants.INTAKE_MOTOR_ID));
    private final Joystick driverJoystick = new Joystick(DriverIOConstants.DRIVER_CONTROLLER_PORT);
    private final Joystick operatorJoystick = new Joystick(OperatorIOConstants.OPERATOR_CONTROLLER_PORT);

    // Driver auto-align
    private final JoystickButton alignToAmpButton = new JoystickButton(driverJoystick,
            DriverIOConstants.ALIGN_TO_AMP_BUTTON_ID);
    private final JoystickButton alignToSourceButton = new JoystickButton(driverJoystick,
            DriverIOConstants.ALIGN_TO_SOURCE_BUTTON_ID);
    private final JoystickButton alignToSpeakerButton = new JoystickButton(driverJoystick,
            DriverIOConstants.ALIGN_TO_SPEAKER_BUTTON_ID);

    // Driver intake
    private final JoystickButton intakeButton = new JoystickButton(driverJoystick, DriverIOConstants.INTAKE_BUTTON_ID);

    // Flywheel
    private final JoystickButton shooterButton = new JoystickButton(operatorJoystick,
            OperatorIOConstants.SHOOTER_BUTTON_ID);
    private final JoystickButton unshooterButton = new JoystickButton(operatorJoystick,
            OperatorIOConstants.UNSHOOTER_BUTTON_ID);

    // Indexer
    private final JoystickButton outtakeButton = new JoystickButton(operatorJoystick,
            OperatorIOConstants.OUTTAKE_BUTTON_ID);
    private final JoystickButton operatorIndexButton = new JoystickButton(operatorJoystick,
            OperatorIOConstants.INDEXER_BUTTON_ID);
    private final JoystickButton driveIndexButton = new JoystickButton(driverJoystick,
            DriverIOConstants.INDEXER_BUTTON_ID);

    // Arm setpoints
    private final POVButton subwooferButton = new POVButton(operatorJoystick, OperatorIOConstants.SUBWOOFER_POV_ANGLE); // UP
    private final POVButton wingButton = new POVButton(operatorJoystick, OperatorIOConstants.WING_POV_ANGLE); // RIGHT
    private final POVButton podiumButton = new POVButton(operatorJoystick, OperatorIOConstants.PODIUM_POV_ANGLE); // LEFT
    private final POVButton passButton = new POVButton(operatorJoystick, OperatorIOConstants.PASS_POV_ANGLE); // DOWN

    // Arm and flywheel setpoints
    private final JoystickButton ampButton = new JoystickButton(operatorJoystick, OperatorIOConstants.AMP_BUTTON_ID);
    private final JoystickButton sourceButton = new JoystickButton(operatorJoystick,
            OperatorIOConstants.SOURCE_BUTTON_ID); // DOWN
    private final JoystickButton backshotSubwoofer = new JoystickButton(operatorJoystick,
            OperatorIOConstants.SUBWOOFER_BACKSHOT_ID);
    private final JoystickButton backshotPodium = new JoystickButton(operatorJoystick,
            OperatorIOConstants.PODIUM_BACKSHOT_ID);

    private final JoystickButton userSignalButton = new JoystickButton(operatorJoystick,
            OperatorIOConstants.USER_SIGNAL_BUTTON_ID);

    private final AutonSelector<Object> autoChooser = new AutonSelector<>("Auto Chooser", "Do Nothing", List.of(),
            () -> Commands.none());
    private final AutoFactory autoFactory;
    private final RoutineFactory routineFactory;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Robot.isReal()) {
            SwerveModuleIOSparkMax frontLeft = new SwerveModuleIOSparkMax(FrontLeftModuleConstants.moduleID,
                    "Front left ", FrontLeftModuleConstants.angleID, FrontLeftModuleConstants.driveID,
                    FrontLeftModuleConstants.angleOffset, FrontLeftModuleConstants.inverted);
            SwerveModuleIOSparkMax frontRight = new SwerveModuleIOSparkMax(FrontRightModuleConstants.moduleID,
                    " Front right", FrontRightModuleConstants.angleID, FrontRightModuleConstants.driveID,
                    FrontRightModuleConstants.angleOffset, FrontRightModuleConstants.inverted);
            SwerveModuleIOSparkMax backLeft = new SwerveModuleIOSparkMax(BackLeftModuleConstants.moduleID, " Back left",
                    BackLeftModuleConstants.angleID, BackLeftModuleConstants.driveID,
                    BackLeftModuleConstants.angleOffset, BackLeftModuleConstants.inverted);
            SwerveModuleIOSparkMax backRight = new SwerveModuleIOSparkMax(BackRightModuleConstants.moduleID,
                    "Back right", BackRightModuleConstants.angleID, BackRightModuleConstants.driveID,
                    BackRightModuleConstants.angleOffset, BackRightModuleConstants.inverted);

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

            driveBase = new DriveBase(new GyroIO() {}, new Vision(new VisionIOSim()), frontLeft, frontRight, backLeft,
                    backRight, false);
            pivot = new Pivot(new PivotIOSim());
            shooter = new Shooter(new ShooterIOSim());
        }

        this.autoFactory = new AutoFactory(driveBase, shooter, intake, pivot, indexer, autoChooser::getResponses);
        this.routineFactory = new RoutineFactory(autoFactory);
        registerNamedCommands();
        setDefaultCommands();
        smartDashSetup();
        configureButtonBindings();
    }

    private void setDefaultCommands() {
        // intake.setDefaultCommand(new StopIntakeCommand(intake));
        // indexer.setDefaultCommand(new CenterNoteCommand(indexer).unless(() -> !indexer.flywheelBeamBroken() && !indexer.intakeBeamBroken()));
        indexer.setDefaultCommand(new SpinIndexerCommand(indexer, 0));
        driveBase.setDefaultCommand(new SwerveDriveCommand(driveBase,
                () -> -driverJoystick.getRawAxis(DriverIOConstants.STRAFE_Y_AXIS),
                () -> -driverJoystick.getRawAxis(DriverIOConstants.STRAFE_X_AXIS),
                () -> -driverJoystick.getRawAxis(DriverIOConstants.ROTATION_AXIS), () -> DriveConstants.FIELD_CENTRIC));
        pivot.setDefaultCommand(
                new RotatePivotCommand(pivot,
                        () -> pivot.getPosition().getDegrees() + -20 * MathUtil.applyDeadband(
                                operatorJoystick.getRawAxis(OperatorIOConstants.PIVOT_ANGLE_AXIS),
                                PivotConstants.INPUT_DEADBAND)));
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("Begin Intake", autoFactory.intake());
        NamedCommands.registerCommand("Shoot", autoFactory.aimAndShoot());
        NamedCommands.registerCommand("Shoot Turn And Intake", autoFactory.shootTurnIntake());
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
        alignToAmpButton.whileTrue(new TurnToAngleCommand(driveBase,
                                () -> AlliancePoseMirror.flipRotation(AlliancePoseMirror.mirrorRotation2d(FieldConstants.BLUE_ALLIANCE_AMP_POSE2D.getRotation())), () -> -driverJoystick.getRawAxis(DriverIOConstants.STRAFE_Y_AXIS),
                () -> -driverJoystick.getRawAxis(DriverIOConstants.STRAFE_X_AXIS),
                                () -> true, false));
        alignToSourceButton.whileTrue(new TurnToAngleCommand(driveBase,
                                () -> AlliancePoseMirror.flipRotation(AlliancePoseMirror.mirrorRotation2d(FieldConstants.BLUE_ALLIANCE_SOURCE_POSE2D.getRotation())), () -> driverJoystick.getRawAxis(DriverIOConstants.STRAFE_Y_AXIS),
                () -> -driverJoystick.getRawAxis(DriverIOConstants.STRAFE_X_AXIS),
                                () -> true, false));
        alignToSpeakerButton.whileTrue(new TurnToPointCommand(driveBase, driveBase::getPose,
                FieldConstants.BLUE_ALLIANCE_SPEAKER_POSE3D.toPose2d(),
                () -> driverJoystick.getRawAxis(DriverIOConstants.STRAFE_Y_AXIS),
                () -> -driverJoystick.getRawAxis(DriverIOConstants.STRAFE_X_AXIS), () -> DriveConstants.FIELD_CENTRIC,
                false).alongWith(autoFactory.autoAimHold()));
        intakeButton.whileTrue(new IntakeNoteCommand(indexer, intake)
                        .alongWith(new RotatePivotCommand(pivot, 0))
                        .alongWith(new InstantCommand(() -> shooter.setIdleMode(NeutralModeValue.Brake))));
        driveIndexButton.whileTrue(new PeriodicConditionalCommand(
                new SpinIndexerCommand(indexer, IndexerConstants.FAST_INDEXER_MOTOR_SPEED),
                new SpinIndexerCommand(indexer, IndexerConstants.FAST_INDEXER_MOTOR_SPEED),
                () -> shooter.getLowerFlywheelVelocityRPS() > shooter.getSetpoint()
                        * ShooterConstants.SHOOTING_FLYWHEEL_VELOCITY_DEADBAND_FACTOR
                        && shooter.getUpperFlywheelVelocityRPS() > shooter.getSetpoint()
                                * ShooterConstants.SHOOTING_FLYWHEEL_VELOCITY_DEADBAND_FACTOR));
        operatorIndexButton.whileTrue(new PeriodicConditionalCommand(
                new SpinIndexerCommand(indexer, IndexerConstants.FAST_INDEXER_MOTOR_SPEED),
                new SpinIndexerCommand(indexer, IndexerConstants.FAST_INDEXER_MOTOR_SPEED),
                () -> shooter.getLowerFlywheelVelocityRPS() > shooter.getSetpoint()
                        * ShooterConstants.SHOOTING_FLYWHEEL_VELOCITY_DEADBAND_FACTOR
                        && shooter.getUpperFlywheelVelocityRPS() > shooter.getSetpoint()
                                * ShooterConstants.SHOOTING_FLYWHEEL_VELOCITY_DEADBAND_FACTOR));
        shooterButton.whileTrue(
                new SpinShooterCommand(shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED, false));
        unshooterButton.whileTrue(
                new SpinShooterCommand(shooter, ShooterConstants.UNSHOOTER_SPEED, ShooterConstants.UNSHOOTER_SPEED, false));
        ampButton.whileTrue(new RotatePivotCommand(pivot, PivotConstants.AMP_ANGLE_SETPOINT)
                .alongWith(new SpinShooterCommand(shooter, ShooterConstants.AMP_SPEED, ShooterConstants.AMP_SPEED, true)));
        outtakeButton.whileTrue(new SpinIntakeCommand(intake, IntakeConstants.OUTTAKE_SPEED)
                .alongWith(new SpinIndexerCommand(indexer, IndexerConstants.SLOW_INDEXER_MOTOR_OUTTAKE_SPEED)));
        subwooferButton.whileTrue(new RotatePivotCommand(pivot, PivotConstants.SUBWOOFER_ANGLE_SETPOINT).alongWith(
                new SpinShooterCommand(shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED, false)));
        wingButton.whileTrue(new RotatePivotCommand(pivot, PivotConstants.WING_ANGLE_SETPOINT).alongWith(
                new SpinShooterCommand(shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED, false)));
        podiumButton.whileTrue(new RotatePivotCommand(pivot, PivotConstants.PODIUM_ANGLE_SETPOINT).alongWith(
                new SpinShooterCommand(shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED, false)));
        passButton.whileTrue(new RotatePivotCommand(pivot, PivotConstants.PASS_ANGLE_SETPOINT)
                .alongWith(new SpinShooterCommand(shooter, ShooterConstants.PASS_SPEED, ShooterConstants.PASS_SPEED, false)));
        backshotPodium.whileTrue(new RotatePivotCommand(pivot, PivotConstants.BACKSHOT_PODIUM_ANGLE_SETPOINT)
                .alongWith(new SpinShooterCommand(shooter, ShooterConstants.PASS_SPEED, ShooterConstants.PASS_SPEED, false)));
        backshotSubwoofer.whileTrue(new RotatePivotCommand(pivot, PivotConstants.BACKSHOT_SUBWOOFER_ANGLE_SETPOINT)
                .alongWith(new SpinShooterCommand(shooter, ShooterConstants.PASS_SPEED, ShooterConstants.PASS_SPEED, false)));
        sourceButton.whileTrue(new RotatePivotCommand(pivot, PivotConstants.SOURCE_PICKUP_ANGLE_SETPOINT).alongWith(
                new SpinShooterCommand(shooter, ShooterConstants.UNSHOOTER_SPEED, ShooterConstants.UNSHOOTER_SPEED, false)));
        userSignalButton.onTrue(new InstantCommand(() -> leds.setUserSignal(true)).ignoringDisable(true))
                .onFalse(new InstantCommand(() -> leds.setUserSignal(false)).ignoringDisable(true));
    }

    public boolean getOperatorConnected() {
        return operatorJoystick.isConnected();
    }
 
    public boolean getDriverConnected() {
        return driverJoystick.isConnected();
    }

    public void smartDashSetup() {

        autoChooser.addRoutine("Characterize",
                List.of(new AutoQuestion<>("Which Subsystem?", Map.of("DriveBase", driveBase, "Pivot", pivot)),
                        new AutoQuestion<>("Which Routine",
                                Map.of("Quasistatic Foward", CharacterizationRoutine.QUASISTATIC_FORWARD,
                                        "Quasistatic Backward", CharacterizationRoutine.QUASISTATIC_BACKWARD,
                                        "Dynamic Forward", CharacterizationRoutine.DYNAMIC_FORWARD, "Dynamic Backward",
                                        CharacterizationRoutine.DYNAMIC_BACKWARD))),
                autoFactory::getCharacterizationRoutine);

        // autoChooser.addRoutine("Drive", List.of(new AutoQuestion<>("Starting Pose?", Map.of("Angled Amp Side", 0, "Subwoofer Center", 1, "Angled Source Side", 2))), autoFactory::getDriveAuto);
        autoChooser.addRoutine("Drive", List.of(), autoFactory::getDriveAuto);
        autoChooser.addRoutine("Score Preload", List.of(), autoFactory::getScoreAuto);
        autoChooser.addRoutine("Score Preload And Drive", List.of(), autoFactory::getScoreAndDriveAuto);
        autoChooser.addRoutine("Score Preload and Drive Out (Amp Side)", List.of(), autoFactory::getScoreAndDriveAutoAmpSide);
        autoChooser.addRoutine("2 Note Subwoofer Center Auto", List.of(), autoFactory::getTwoNote);
        autoChooser.addRoutine("Cleanup Auto", List.of(), routineFactory::getCleanup);

        autoChooser.addRoutine("Wing And Midline Auto", List.of(
                new AutoQuestion<>("Starting Note?", Map.of("Wing Right", 0, "Wing Center", 1, "Wing Left", 2)),
                new AutoQuestion<>("Last Wing Note?", Map.of("-Wing Right", 0, "-Wing Center", 1, "-Wing Left", 2)),
                new AutoQuestion<>("Starting Center Line Note?",
                        Map.of("Midline Left", 0, "Midline Center Left", 1, "Midline Center", 2, "Midline Center Right",
                                3, "Midline Right (Source Side)", 4)),
                new AutoQuestion<>("Starting Center Line Note?",
                        Map.of("-Midline Left", 0, "-Midline Center Left", 1, "-Midline Center", 2,
                                "-Midline Center Right", 3, "-Midline Right (Source Side)", 4))),
                autoFactory::getWingAndMidlineAuto);
    }

    public void setIdleMode(IdleMode idleMode, NeutralModeValue shooterIdleMode) {
        driveBase.setIdleMode(idleMode);
        pivot.setIdleMode(idleMode);
        intake.setIdleMode(idleMode);
        shooter.setIdleMode(shooterIdleMode);
    }
}
