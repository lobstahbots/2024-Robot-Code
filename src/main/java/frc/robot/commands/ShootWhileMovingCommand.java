
//Credit to 6328 Mechanical Advantage (https://github.com/Mechanical-Advantage/RobotCode2024/blob/0696b49e810ac2eb92b5f7aebe9a82a7fed6897c/src/main/java/org/littletonrobotics/frc2024/util/shooting/ShotCalculator.java) 

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.DriveBase;

public class ShootWhileMovingCommand extends Command {
  private final DriveBase driveBase;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final ChassisSpeeds chassisSpeeds;
  private final Translation2d target;
  private final boolean reversed;
  private final boolean velocityCorrection;
  private double distanceToTarget;
  private final PIDController aimPidControllerFront = new PIDController(1, 0, 0);
  private final PIDController aimPidControllerRear = new PIDController(1, 0, 0);

 /**
   * Aim robot at a desired point on the field
   * @param robotPoseSupplier A robot supplier 
   * @param velocityCorrection Turns velocity correction on/off
   * @param point Target point, pass in origin to signify invalid point
   */
  public ShootWhileMovingCommand(DriveBase driveBase, Supplier<Pose2d> robotPoseSupplier, ChassisSpeeds robotSpeeds, Translation2d point, boolean reversed, boolean velocityCorrection) {
    this.driveBase = driveBase;
    this.robotPoseSupplier = robotPoseSupplier;
    this.chassisSpeeds = robotSpeeds;
    this.target = point;
    this.reversed = reversed;
    this.velocityCorrection = velocityCorrection;
  }

  @Override
  public void execute() {
    // Return if invalid point
    if (target == null) distanceToTarget = 0.0;
    // Angle to target point
    Pose2d robotPose = robotPoseSupplier.get();
    Rotation2d targetAngle = new Rotation2d(target.getX() - robotPose.getX(), target.getY() - robotPose.getY());
    // Movement vector of robot
    double velocityOutput = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Translation2d robotVector = new Translation2d(velocityOutput * robotPose.getRotation().getCos(), velocityOutput * robotPose.getRotation().getSin());
    // Aim point
    Translation2d aimPoint = target.minus(new Translation2d(robotVector.getX(), robotVector.getY()));
    // Vector from robot to target
    Translation2d targetVector = new Translation2d(robotPose.getTranslation().getDistance(target) * targetAngle.getCos(), robotPose.getTranslation().getDistance(target) * targetAngle.getSin());
    // Parallel component of robot's motion to target vector
    Translation2d parallelRobotVector = targetVector.times(Math.pow(robotVector.getNorm() * targetVector.getNorm() * robotVector.minus(targetVector).getAngle().getCos() / targetVector.getNorm(), 2));
    // Perpendicular component of robot's motion to target vector
    Translation2d perpendicularRobotVector = robotVector.minus(parallelRobotVector).times(velocityCorrection ? SwerveConstants.AIM_VELOCITY_COMPENSATION_DEADBAND : 0.0);
    // Adjust aim point using calculated vector
    Translation2d adjustedPoint = target.minus(new Translation2d(perpendicularRobotVector.getX(), perpendicularRobotVector.getY()));
    // Calculate new angle using adjusted point
    Rotation2d adjustedAngle = new Rotation2d(adjustedPoint.getX() - robotPose.getX(), adjustedPoint.getY() - robotPose.getY());
    // Calculate necessary rotate rate
    double rotateOutput = reversed
      ? aimPidControllerRear.calculate(robotPose.getRotation().plus(new Rotation2d(Math.PI)).getDegrees(), adjustedAngle.getDegrees())
      : aimPidControllerFront.calculate(robotPose.getRotation().getDegrees(), adjustedAngle.getDegrees());

    // Log aim point
    Logger.recordOutput("AimPoint", new Pose2d(aimPoint, new Rotation2d()));

    // Drive robot accordingly
    double moveDirection = Math.atan2(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond);

    driveBase.driveRobotRelative(
      new ChassisSpeeds(Units.MetersPerSecond.of(-velocityOutput * Math.cos(moveDirection)),
      Units.MetersPerSecond.of(-velocityOutput * Math.sin(moveDirection)),
      Units.DegreesPerSecond.of(rotateOutput))
    );

    distanceToTarget = robotPose.getTranslation().getDistance(aimPoint);
  }
    /**
   * Get inertial velocity of robot
   * @return Inertial velocity of robot in m/s
   */
  public Measure<Velocity<Distance>> getInertialVelocity() {
    return Units.MetersPerSecond.of(
      Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
    );
  }

  @Override 
  public boolean isFinished() {
    return distanceToTarget == 0.0;
  }
}