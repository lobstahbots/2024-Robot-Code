// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.vision.PhotonVision;
import stl.sysId.CharacterizableSubsystem;

public class DriveBase extends CharacterizableSubsystem {
  /** Creates a new SwerveDriveBase. */
  private final SwerveModule[] modules; 

  private SwerveDrivePoseEstimator swerveOdometry;
  private final GyroIO gyro;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private boolean isOpenLoop;
  private Rotation2d simRotation = new Rotation2d();
  private final PhotonVision photonVision;

  private SwerveModuleState[] setpointStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  private Field2d field;

  public DriveBase(GyroIO gyroIO, PhotonVision photonVision, SwerveModuleIO frontLeft, SwerveModuleIO frontRight, SwerveModuleIO backLeft, SwerveModuleIO backRight, boolean isOpenLoop) {

    this.modules = new SwerveModule[]{new SwerveModule(frontLeft, 0), new SwerveModule(frontRight, 1), new SwerveModule(backLeft, 2), new SwerveModule(backRight, 3)};
    
    this.gyro = gyroIO;

    gyro.zeroGyro();
    this.photonVision = photonVision;
    swerveOdometry = new SwerveDrivePoseEstimator(DriveConstants.KINEMATICS, gyroInputs.yawPosition, getPositions(), new Pose2d());

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    this.isOpenLoop = isOpenLoop;

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(1, 0.0, 0.0), // Translation PID constants
            new PIDConstants(1, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {return DriverStation.getAlliance().get() ==  DriverStation.Alliance.Red;},
      this 
    );
  }

  /**Resets pose of odometry to a given pose.
   * 
   * @param pose The desired pose to reset the odometry to.
   */
  public void resetPose(Pose2d pose) {
    swerveOdometry.resetPosition(gyroInputs.yawPosition, getPositions(), pose);
  }
  
  /**Gets pose from odometry.
   * 
   * @return The current estimated pose of the odometry
   */
  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  /**Gets states of the four swerve modules.
   * 
   * @return The states of the four swerve modules in a {@link SwerveModuleState} array.
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule module : modules) {
      states[module.getModuleID()] = module.getState();
    }
    return states;
  }

  /**Gets positions of the four swerve modules.
   * 
   * @return The positions of the four swerve modules in a {@link SwerveModulePosition} array.
   */
  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule module : modules) {
      positions[module.getModuleID()] = module.getPosition();
    }
    return positions;
  }
 
  /**Gets robot relative ChassisSpeeds.
   * 
   * @return The robot-relative {@link ChassisSpeeds}.
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
  return ChassisSpeeds.fromFieldRelativeSpeeds(DriveConstants.KINEMATICS.toChassisSpeeds(getStates()), gyroInputs.yawPosition);
 } 

/**
 * Drives the robot robot-relative according to provided {@link ChassisSpeeds}.
 * 
 * @param chassisSpeeds The desired ChassisSpeeds. Should be robot relative.
 */
public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
  ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
  SwerveModuleState[] newStates = DriveConstants.KINEMATICS.toSwerveModuleStates(discreteSpeeds);
  SwerveDriveKinematics.desaturateWheelSpeeds(newStates, DriveConstants.MAX_DRIVE_SPEED);
  setModuleStates(newStates);
}

/**
 * Drives the robot field-relative according to provided {@link ChassisSpeeds}.
 * 
 * @param chassisSpeeds The desired ChassisSpeeds. Should be robot relative.
 */
public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
  ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
  SwerveModuleState[] newStates = DriveConstants.KINEMATICS.toSwerveModuleStates(getFieldRelativeChassisSpeeds(discreteSpeeds));
  SwerveDriveKinematics.desaturateWheelSpeeds(newStates, DriveConstants.MAX_DRIVE_SPEED);
  setModuleStates(newStates);
}

/**Sets desired SwerveModuleStates. Optimizes states.
 * 
 * @param desiredStates The states to set for each module.
 * @return The optimized SwerveModuleStates, now desired states.
 */
public SwerveModuleState[] setModuleStates(SwerveModuleState[] desiredStates) {
  SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
  SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, DriveConstants.MAX_DRIVE_SPEED);
  for(SwerveModule module: modules) {
    optimizedStates[module.getModuleID()] = module.setDesiredState(desiredStates[module.getModuleID()], isOpenLoop);
    setpointStates = optimizedStates;
  }
  return optimizedStates;
}

/**Converts robot relative {@link ChassisSpeeds} to field relative.
 * 
 * @param robotRelativeSpeeds The robot relative speeds to convert
 * @return The field relative ChassisSpeeds.
 */
public ChassisSpeeds getFieldRelativeChassisSpeeds(ChassisSpeeds robotRelativeSpeeds) {
  Rotation2d angle = new Rotation2d();
  if(Robot.isSimulation()) {
    angle = simRotation;
  } else {
    angle = getPose().getRotation();
  }
  return new ChassisSpeeds(
    robotRelativeSpeeds.vxMetersPerSecond * angle.getCos()
                  - robotRelativeSpeeds.vyMetersPerSecond * angle.getSin(),
                  robotRelativeSpeeds.vyMetersPerSecond * angle.getCos()
                  + robotRelativeSpeeds.vxMetersPerSecond * angle.getSin(),
                  robotRelativeSpeeds.omegaRadiansPerSecond);
}

/**Sets the {@link IdleMode} of the DriveBase motors.
 * 
 * @param mode The braking mode (Coast or Brake) of the swerve module motors.
 */
public void setBrakingMode(IdleMode mode) {
  for(SwerveModule module: modules) {
    module.setBrakingMode(mode);
  }
}

/**Stops all of the modules' motors. */
public void stopMotors() {
  for(SwerveModule module: modules) {
    module.stop();
  }
}

/**
 * @return Whether or not the controller is open loop.
 */
public boolean isOpenLoop() {
  return isOpenLoop;
}

/**Sets whether the controller is open loop. 
 * 
 * @param newValue The new boolean to set.
*/
public void setIsOpenLoop(boolean newValue) {
  isOpenLoop = newValue;
}

/**
 * Gets gyro angle
 * @return The angle of the gyro as a {@link} Rotation2d.
 */
public Rotation2d getGyroAngle() {
  return gyroInputs.yawPosition;
}

@Override
/**Runs motors during characterization voltage ramp routines.*/
public void runVolts(double volts) {
  for(SwerveModule module: modules) {
    module.runVolts(volts);
  }
}

  @Override
  public void periodic() {
    if(Robot.isSimulation()) {
      var twist = DriveConstants.KINEMATICS.toTwist2d(getPositions());
      simRotation = Rotation2d.fromDegrees(twist.dtheta);
      SmartDashboard.putNumber("Twist Theta", twist.dtheta);
      swerveOdometry.update(simRotation, getPositions());
    } else {
      swerveOdometry.update(gyroInputs.yawPosition, getPositions());
    }
    Optional<Pose2d> estimatedPose = photonVision.getEstimatedPose(getPose());
    if (estimatedPose.isPresent()) swerveOdometry.addVisionMeasurement(estimatedPose.get(), photonVision.getTimestamp());
    field.setRobotPose(getPose());
    SmartDashboard.putString("Pose", getPose().toString());
    Logger.recordOutput("Odometry", getPose());
     gyro.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    SmartDashboard.putBoolean("Field Centric", DriveConstants.FIELD_CENTRIC);
    for (var module : modules) {
      module.periodic();
    }
  // Clear setpoint logs
       Logger.recordOutput("SwerveStates/Desired", new double[] {});
    if (DriverStation.isDisabled()) {
      // Stop moving while disabled
      for (var module : modules) {
        module.stop();
      }
    }

    else {
    Logger.recordOutput("SwerveStates/Measured", getStates());
    Logger.recordOutput("SwerveStates/Desired", setpointStates);
    Logger.recordOutput("Odometry/Robot", getPose());
    
    //Log 3D odometry pose
    Pose3d robotPose3d = new Pose3d(getPose());
    robotPose3d =
        robotPose3d
            .exp(
                new Twist3d(
                    0.0,
                    0.0,
                    Math.abs(gyroInputs.pitchPosition.getRadians()) * RobotConstants.TRACK_WIDTH / 2.0,
                    0.0,
                    gyroInputs.pitchPosition.getRadians(),
                    0.0))
            .exp(
                new Twist3d(
                    0.0,
                    0.0,
                    Math.abs(gyroInputs.rollPosition.getRadians()) * RobotConstants.TRACK_WIDTH / 2.0,
                    gyroInputs.rollPosition.getRadians(),
                    0.0,
                    0.0));

    Logger.recordOutput("Odometry/Robot3d", robotPose3d);
    }
  }
}
