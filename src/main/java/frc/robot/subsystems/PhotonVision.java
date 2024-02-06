package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class PhotonVision extends SubsystemBase {
    private final PhotonVisionIO io;
    private final PhotonVisionIOInputsAutoLogged inputs = new PhotonVisionIOInputsAutoLogged();
    
    public PhotonVision(PhotonVisionIO io) {
       this.io = io;
    }

    /**
     * Get the estimated pose from both cameras.
     * @return The estimated pose.
     */
    public Optional<Pose2d> getEstimatedPose(Pose2d odometryPose) {
        Pose2d averagePose = null;
        double sumX = 0;
        double sumY = 0;
        Rotation2d sumRotation = new Rotation2d();
        double sumConfidence = 0;
        Pose2d frontPose = inputs.estimatedFrontPose.toPose2d();
        if (inputs.frontConfidence > VisionConstants.POSE_CONFIDENCE_FILTER_THRESHOLD && frontPose.minus(odometryPose).getTranslation().getNorm() < VisionConstants.VISION_ODOMETRY_DIFFERENCE_FILTER_THRESHOLD) {
            sumX += frontPose.getX() * inputs.frontConfidence;
            sumY += frontPose.getY() * inputs.frontConfidence;
            sumRotation = sumRotation.plus(frontPose.getRotation().times(inputs.frontConfidence));
            sumConfidence += inputs.frontConfidence;
        }
        Pose2d rearPose = inputs.estimatedRearPose.toPose2d();
        if (inputs.rearConfidence > VisionConstants.POSE_CONFIDENCE_FILTER_THRESHOLD && rearPose.minus(odometryPose).getTranslation().getNorm() < VisionConstants.VISION_ODOMETRY_DIFFERENCE_FILTER_THRESHOLD) {
            sumX += rearPose.getX() * inputs.rearConfidence;
            sumY += rearPose.getY() * inputs.rearConfidence;
            sumRotation = sumRotation.plus(rearPose.getRotation().times(inputs.rearConfidence));
            sumConfidence += inputs.rearConfidence;
        }

        if (sumConfidence != 0) averagePose = new Pose2d(sumX, sumY, sumRotation).div(sumConfidence);
        return Optional.of(averagePose);
    }

    /**
     * Get the tracked targets from the front camera.
     * @return The tracked targets.
     */
    public List<PhotonTrackedTarget> getFrontTargets() {
        return inputs.frontTargets;
    }

    /**
     * Get the tracked targets from the rear camera.
     * @return The tracked targets.
     */
    public List<PhotonTrackedTarget> getRearTargets() {
        return inputs.rearTargets;
    }

    /**
     * Get the fiducial IDs of the targets in the front camera.
     * @return A list of the IDs.
     */
    public List<Integer> getFrontFiducialIDs() {
        return inputs.visibileFrontFiducialIDs;
    }

    /**
     * Get the fiducial IDs of the targets in the rear camera.
     * @return A list of the IDs.
     */
    public List<Integer> getRearFiducialIDs() {
        return inputs.visibleRearFiducialIDs;
    }
}
