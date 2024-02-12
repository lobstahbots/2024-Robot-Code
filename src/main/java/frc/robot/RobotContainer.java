// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.DriveConstants.BackLeftModuleConstants;
import frc.robot.Constants.DriveConstants.BackRightModuleConstants;
import frc.robot.Constants.DriveConstants.FrontLeftModuleConstants;
import frc.robot.Constants.DriveConstants.FrontRightModuleConstants;
import frc.robot.TrajectoryFactory.PathType;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.GyroIO;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.PhotonVisionReal;
import frc.robot.subsystems.SwerveModuleReal;
import frc.robot.subsystems.SwerveModuleSim;

import java.util.function.Function;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveBase driveBase;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final Joystick driverJoystick =
      new Joystick(IOConstants.DRIVER_CONTROLLER_PORT);
  
  private final TrajectoryFactory trajectoryFactory = new TrajectoryFactory();

  private final LoggedDashboardChooser<Pose2d> startingPositionChooser = new LoggedDashboardChooser<>("Starting Position");
  private final LoggedDashboardChooser<Function<Pose2d, Command>> autoChooser = new LoggedDashboardChooser<>("Auto Chooser");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if(Robot.isReal()) {
      SwerveModuleReal frontLeft = new SwerveModuleReal(FrontLeftModuleConstants.moduleID, FrontLeftModuleConstants.angleID, FrontLeftModuleConstants.driveID, FrontLeftModuleConstants.angleOffset, FrontLeftModuleConstants.inverted);
      SwerveModuleReal frontRight = new SwerveModuleReal(FrontRightModuleConstants.moduleID, FrontRightModuleConstants.angleID, FrontRightModuleConstants.driveID, FrontRightModuleConstants.angleOffset, FrontRightModuleConstants.inverted);
      SwerveModuleReal backLeft = new SwerveModuleReal(BackLeftModuleConstants.moduleID, BackLeftModuleConstants.angleID, BackLeftModuleConstants.driveID, BackLeftModuleConstants.angleOffset, BackLeftModuleConstants.inverted);
      SwerveModuleReal backRight = new SwerveModuleReal(BackRightModuleConstants.moduleID, BackRightModuleConstants.angleID, BackRightModuleConstants.driveID, BackRightModuleConstants.angleOffset, BackRightModuleConstants.inverted);
  
      driveBase = new DriveBase(new NavXGyro(), new PhotonVision(new PhotonVisionReal()), frontLeft, frontRight, backRight, backLeft, false);
    } 
    else {
      SwerveModuleSim frontLeft = new SwerveModuleSim(FrontLeftModuleConstants.angleOffset);
      SwerveModuleSim frontRight = new SwerveModuleSim(FrontRightModuleConstants.angleOffset);
      SwerveModuleSim backLeft = new SwerveModuleSim(BackLeftModuleConstants.angleOffset);
      SwerveModuleSim backRight = new SwerveModuleSim(BackRightModuleConstants.angleOffset);

      driveBase = new DriveBase(new GyroIO(){}, new PhotonVision(new PhotonVisionReal()), frontLeft, frontRight, backLeft, backRight, false);
    }
   
    setTeleopDefaultCommands();

    smartDashSetup();
  }

  private void setTeleopDefaultCommands() {
    driveBase.setDefaultCommand(
      new SwerveDriveCommand(driveBase,
          () -> driverJoystick.getRawAxis(IOConstants.STRAFE_Y_AXIS),
          () -> -driverJoystick.getRawAxis(IOConstants.STRAFE_X_AXIS),
          () -> driverJoystick.getRawAxis(IOConstants.ROTATION_AXIS),
          DriveConstants.FIELD_CENTRIC));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get().apply(startingPositionChooser.get());
  }

  public void setAutonDefaultCommands() {
    driveBase.setBrakingMode(IdleMode.kBrake);
  }

  public void configureButtonBindings() {
  }

  protected Command getSimpleAuto(Pose2d startingPosition) {
    String pathname = "";
    if (startingPosition == PathConstants.STATION_1) {
      pathname = "Station1SimpleAuto";
    } else if (startingPosition == PathConstants.STATION_2) {
      pathname = "Station2SimpleAuto";
    } else if (startingPosition == PathConstants.STATION_3) {
      pathname = "Station3SimpleAuto";
    }
    return trajectoryFactory.getPathFindToPathCommand(pathname, PathType.CHOREO);
  }

  public void smartDashSetup() {
    autoChooser.addDefaultOption("Do Nothing", pose -> new WaitUntilCommand(() -> false));
    autoChooser.addOption("Simple Auto", this::getSimpleAuto);

    startingPositionChooser.addDefaultOption("Station 1", PathConstants.STATION_1);
    startingPositionChooser.addOption("Station 2", PathConstants.STATION_2);
    startingPositionChooser.addOption("Station 3", PathConstants.STATION_3);
  }
}
