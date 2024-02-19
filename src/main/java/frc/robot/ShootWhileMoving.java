package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShootWhileMoving {
    public record ShotData(
        double effectiveRobotToSpeakerDist,
        double radialFeedforward, // ff value due to radial velocity of robot to speaker
        Rotation2d goalHeading) {} // heading of robot to match tangential velocity

    public ShotData calculate(
      Translation2d speakerTranslation, Translation2d robotTranslation, Translation2d linearFieldVelocity, double shotTime) {
    // Calculate radial and tangential velocity from speaker
    Rotation2d speakerToRobotAngle = robotTranslation.minus(speakerTranslation).getAngle();
    Translation2d tangentialVelocity =
        linearFieldVelocity.rotateBy(speakerToRobotAngle.unaryMinus());
    // Positive when velocity is away from speaker
    double radialComponent = tangentialVelocity.getX();
    // Positive when traveling CCW about speaker
    double tangentialComponent = tangentialVelocity.getY();

    // Add robot velocity to raw shot speed
    double rawDistToGoal = robotTranslation.getDistance(speakerTranslation);
    double shotSpeed = rawDistToGoal / shotTime + radialComponent;
    if (shotSpeed <= 0.0) shotSpeed = 0.0;
    // Rotate back into field frame then add take opposite
    Rotation2d goalHeading =
        new Pose2d(robotTranslation.getX(), robotTranslation.getY(), robotTranslation.getAngle().unaryMinus()).transformBy(new Transform2d(speakerTranslation.getX(), speakerTranslation.getY(), speakerTranslation.getAngle())).getTranslation().getAngle();
    // Aim opposite of tangentialComponent (negative lead when tangentialComponent is positive)
    goalHeading = goalHeading.plus(new Rotation2d(shotSpeed, tangentialComponent));
    double effectiveDist = shotTime * Math.hypot(tangentialComponent, shotSpeed);

    Logger.recordOutput("ShootWhileMoving/heading", goalHeading);
    Logger.recordOutput("ShootWhileMoving/driveFeedVelocity", radialComponent);
    Logger.recordOutput("ShootWhileMoving/effectiveDistance", effectiveDist);
    // Use radial component of velocity for ff value
    return new ShotData(effectiveDist, radialComponent, goalHeading);
  }
}
