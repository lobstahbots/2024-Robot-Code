// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AutoFactory.CharacterizationRoutine;
import frc.robot.Constants.AlertConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.TurnToPointCommand;
import frc.robot.networkalerts.Alert;
import frc.robot.networkalerts.Alert.AlertType;
import frc.robot.auto.AutonSelector;
import frc.robot.auto.AutonSelector.AutoQuestion;
import frc.robot.commands.MoveClimberCommand;
import frc.robot.commands.RotatePivotCommand;
import frc.robot.commands.SpinIntakeCommand;
import frc.robot.commands.SpinShooterCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberSparkMax;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.NavXGyro;
import frc.robot.subsystems.drive.SwerveModuleReal;
import frc.robot.subsystems.drive.SwerveModuleSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotKinematics;
import frc.robot.subsystems.pivot.PivotSim;
import frc.robot.subsystems.pivot.PivotSparkMax;
import frc.robot.subsystems.shooter.ShooterSparkMax;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.subsystems.vision.PhotonVisionReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSparkMax;

import com.pathplanner.lib.auto.NamedCommands;
import java.util.List;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final Climber climber = new Climber(
      new ClimberSparkMax(ClimberConstants.LEFT_CLIMBER_ID, ClimberConstants.RIGHT_CLIMBER_ID));
  private final Intake intake = new Intake(new IntakeSparkMax(IntakeConstants.INTAKE_MOTOR_ID));

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick driverJoystick = new Joystick(IOConstants.DRIVER_CONTROLLER_PORT);
  private final Shooter shooter = new Shooter(new ShooterSparkMax(ShooterConstants.UPPER_SHOOTER_ID, ShooterConstants.LOWER_SHOOTER_ID));
  private final JoystickButton alignToAmpButton = new JoystickButton(driverJoystick,
      IOConstants.ALIGN_TO_AMP_BUTTON_ID);
  private final JoystickButton alignToSourceButton = new JoystickButton(driverJoystick,
      IOConstants.ALIGN_TO_SOURCE_BUTTON_ID);
  private final JoystickButton alignToSpeakerButton = new JoystickButton(driverJoystick,
      IOConstants.ALIGN_TO_SPEAKER_BUTTON_ID);
  private final JoystickButton driveToggleButton = new JoystickButton(driverJoystick, IOConstants.TOGGLE_DRIVE_CENTRICITY_BUTTON_ID);
  private final Joystick operatorJoystick = new Joystick(IOConstants.OPERATOR_CONTROLLER_PORT);
  private final JoystickButton shooterButton = new JoystickButton(operatorJoystick, IOConstants.SHOOTER_BUTTON_ID);
  private final JoystickButton intakeButton = new JoystickButton(operatorJoystick, IOConstants.INTAKE_BUTTON_ID);
  private final JoystickButton climberUpButton = new JoystickButton(operatorJoystick, IOConstants.CLIMBERUP_BUTTON_ID);
  private final JoystickButton climberDownButton = new JoystickButton(operatorJoystick,
      IOConstants.CLIMBERDOWN_BUTTON_ID);
  private final JoystickButton retractPivotButton = new JoystickButton(operatorJoystick, IOConstants.RESET_PIVOT_ANGLE_BUTTON_ID);
  private final JoystickButton slowdownButton = new JoystickButton(driverJoystick, IOConstants.SLOWDOWN_BUTTON_ID);
  
  private final AutonSelector<Object> autoChooser = new AutonSelector<>("Auto Chooser", "Do Nothing", List.of(),
      () -> Commands.none());
  private final AutoFactory autoFactory;

  private final Alert endgameAlert1 = new Alert(String.format("Endgame started - %d seconds remaining", AlertConstants.ENDGAME_ALERT_1_TIME), AlertType.INFO);
  private final Alert endgameAlert2 = new Alert(String.format("%d seconds remaining", AlertConstants.ENDGAME_ALERT_2_TIME), AlertType.INFO);
  private final Alert lowBatteryAlert = new Alert(String.format("Low battery voltage - below %f volts", AlertConstants.LOW_BATTERY_VOLTAGE), AlertType.WARNING);
  private final Alert driverControllerDisconnectedAlert = new Alert("Driver controller disconnected", AlertType.ERROR);
  private final Alert operatorControllerDisconnectedAlert =  new Alert("Operator controller disconnected", AlertType.ERROR);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (Robot.isReal()) {
      SwerveModuleReal frontLeft = new SwerveModuleReal(FrontLeftModuleConstants.moduleID,
          FrontLeftModuleConstants.angleID, FrontLeftModuleConstants.driveID, FrontLeftModuleConstants.angleOffset,
          FrontLeftModuleConstants.inverted);
      SwerveModuleReal frontRight = new SwerveModuleReal(FrontRightModuleConstants.moduleID,
          FrontRightModuleConstants.angleID, FrontRightModuleConstants.driveID, FrontRightModuleConstants.angleOffset,
          FrontRightModuleConstants.inverted);
      SwerveModuleReal backLeft = new SwerveModuleReal(BackLeftModuleConstants.moduleID,
          BackLeftModuleConstants.angleID, BackLeftModuleConstants.driveID, BackLeftModuleConstants.angleOffset,
          BackLeftModuleConstants.inverted);
      SwerveModuleReal backRight = new SwerveModuleReal(BackRightModuleConstants.moduleID,
          BackRightModuleConstants.angleID, BackRightModuleConstants.driveID, BackRightModuleConstants.angleOffset,
          BackRightModuleConstants.inverted);

      driveBase = new DriveBase(new NavXGyro(), new PhotonVision(new PhotonVisionReal()), frontLeft, frontRight,
          backRight, backLeft, false);
      pivot = new Pivot(new PivotSparkMax(PivotConstants.LEFT_MOTOR_ID, PivotConstants.RIGHT_MOTOR_ID));
    } else {
      SwerveModuleSim frontLeft = new SwerveModuleSim(FrontLeftModuleConstants.angleOffset);
      SwerveModuleSim frontRight = new SwerveModuleSim(FrontRightModuleConstants.angleOffset);
      SwerveModuleSim backLeft = new SwerveModuleSim(BackLeftModuleConstants.angleOffset);
      SwerveModuleSim backRight = new SwerveModuleSim(BackRightModuleConstants.angleOffset);

      driveBase = new DriveBase(new GyroIO() {
      }, new PhotonVision(new PhotonVisionReal()), frontLeft, frontRight, backLeft, backRight, false);
      pivot = new Pivot(new PivotSim());
    }

    this.autoFactory = new AutoFactory(driveBase, shooter, intake, pivot, autoChooser::getResponses);

    registerNamedCommands();

    setTeleopDefaultCommands();
    
    smartDashSetup();
  }

  private void setTeleopDefaultCommands() {
    driveBase.setDefaultCommand(
        new SwerveDriveCommand(driveBase,
            () -> driverJoystick.getRawAxis(IOConstants.STRAFE_Y_AXIS),
            () -> -driverJoystick.getRawAxis(IOConstants.STRAFE_X_AXIS),
            () -> driverJoystick.getRawAxis(IOConstants.ROTATION_AXIS),
            () -> DriveConstants.FIELD_CENTRIC));
    pivot.setDefaultCommand(new RotatePivotCommand(pivot, PivotKinematics.getShotAngle(() -> FieldConstants.BLUE_ALLIANCE_SPEAKER_POSE3D.toPose2d(), driveBase::getPose)));
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
        .whileTrue(new TurnToAngleCommand(driveBase, FieldConstants.BLUE_ALLIANCE_AMP_POSE2D.getRotation()));
    alignToSourceButton
        .whileTrue(new TurnToAngleCommand(driveBase, FieldConstants.BLUE_ALLIANCE_SOURCE_POSE2D.getRotation()));
    alignToSpeakerButton
        .whileTrue(new TurnToPointCommand(driveBase::getPose, FieldConstants.BLUE_ALLIANCE_SPEAKER_POSE3D.toPose2d(), driveBase));
    slowdownButton.whileTrue(new SwerveDriveCommand(driveBase,
        () -> DriveConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(IOConstants.STRAFE_Y_AXIS),
        () -> -DriveConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(IOConstants.STRAFE_X_AXIS),
        () -> driverJoystick.getRawAxis(IOConstants.ROTATION_AXIS),
        () -> DriveConstants.FIELD_CENTRIC));
    shooterButton.whileTrue(new SpinShooterCommand(shooter, ShooterConstants.SHOOTER_SPEED, ShooterConstants.SHOOTER_SPEED));
    climberUpButton.whileTrue(new MoveClimberCommand(climber, ClimberConstants.CLIMBER_SPEED));
    climberDownButton.whileTrue(new MoveClimberCommand(climber, -ClimberConstants.CLIMBER_SPEED));
    intakeButton.whileTrue(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_SPEED));
    retractPivotButton.whileTrue(new RotatePivotCommand(pivot, PivotConstants.PIVOT_RESTING_ANGLE));
    driveToggleButton.onTrue(new InstantCommand(() -> DriveConstants.FIELD_CENTRIC = !DriveConstants.FIELD_CENTRIC));
  }

  public void smartDashSetup() {
    autoChooser.addRoutine("Simple Auto", List.of(
        new AutoQuestion<>("Starting Position?", Map.of("Station 1", 1, "Station 2",
            2, "Station 3", 3))),
        autoFactory::getSimpleAuto);

    autoChooser.addRoutine("One-Note Auto", List.of(), autoFactory::getOneNoteAuto);
    autoChooser.addRoutine("Two-Note Auto", List.of(), autoFactory::getTwoNoteAuto);
    autoChooser.addRoutine("Three-Note Auto", List.of(), autoFactory::getThreeNoteAuto);
    autoChooser.addRoutine("Four-Note Auto", List.of(), autoFactory::getFourNoteAuto);

    autoChooser.addRoutine("Characterize", List.of(
        new AutoQuestion<>("Which Subsystem?", Map.of("DriveBase", driveBase, "Pivot", pivot)),
        new AutoQuestion<>("Which Routine", Map.of("Quasistatic Foward", CharacterizationRoutine.QUASISTATIC_FORWARD,
            "Quasistatic Backward", CharacterizationRoutine.QUASISTATIC_BACKWARD, "Dynamic Forward",
            CharacterizationRoutine.DYNAMIC_FORWARD, "Dynamic Backward", CharacterizationRoutine.DYNAMIC_BACKWARD))),
        autoFactory::getCharacterizationRoutine);

    new Trigger(() -> DriverStation.isTeleop() && AlertConstants.ENDGAME_ALERT_2_TIME < DriverStation.getMatchTime() && DriverStation.getMatchTime() < AlertConstants.ENDGAME_ALERT_1_TIME)
        .onTrue(new InstantCommand(() -> endgameAlert1.set(true)))
        .onFalse(new InstantCommand(() -> endgameAlert1.set(false)));
    new Trigger(() -> DriverStation.isTeleop() && 0 < DriverStation.getMatchTime() && DriverStation.getMatchTime() < AlertConstants.ENDGAME_ALERT_2_TIME)
        .onTrue(new InstantCommand(() -> endgameAlert2.set(true)))
        .onFalse(new InstantCommand(() -> endgameAlert2.set(false)));
    new Trigger(() -> RobotController.getBatteryVoltage() < AlertConstants.LOW_BATTERY_VOLTAGE)
        .onTrue(new InstantCommand(() -> lowBatteryAlert.set(true)))
        .onFalse(new InstantCommand(() -> lowBatteryAlert.set(false)));
    new Trigger(driverJoystick::isConnected)
        .onTrue(new InstantCommand(() -> driverControllerDisconnectedAlert.set(false)))
        .onFalse(new InstantCommand(() -> driverControllerDisconnectedAlert.set(true)));
    new Trigger(operatorJoystick::isConnected)
        .onTrue(new InstantCommand(() -> operatorControllerDisconnectedAlert.set(false)))
        .onFalse(new InstantCommand(() -> operatorControllerDisconnectedAlert.set(true)));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("intake", new SpinIntakeCommand(intake, IntakeConstants.INTAKE_SPEED));
    NamedCommands.registerCommand("shoot", autoFactory.getShootCommand());
    NamedCommands.registerCommand("note1pivot", autoFactory.getPivotCommand(AutoConstants.NOTE_1_SHOOTING_ANGLE));
    NamedCommands.registerCommand("note2pivot", autoFactory.getPivotCommand(AutoConstants.NOTE_2_SHOOTING_ANGLE));
    NamedCommands.registerCommand("note3pivot", autoFactory.getPivotCommand(AutoConstants.NOTE_3_SHOOTING_ANGLE));
    NamedCommands.registerCommand("intakePivot", autoFactory.getPivotCommand(AutoConstants.INTAKE_ANGLE));
  }
}
