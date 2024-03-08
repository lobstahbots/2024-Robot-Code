package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
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
     * @param odometryPose The current pose returned by the robot odometry to filter the vision estimate in comparison to.
     * @param gyroRotation The rotation from the gyro; used if there is only one detected tag
     * @return The estimated pose.
     */
    public Optional<Pose2d> getEstimatedPose(Pose2d odometryPose, Rotation2d gyroRotation) {
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

        if (sumConfidence != 0) {
            if (io.getFrontTrackedTargets().size() + io.getRearTrackedTargets().size() > 1) averagePose = new Pose2d(sumX, sumY, sumRotation).div(sumConfidence);
            else averagePose = new Pose2d(sumX, sumY, new Rotation2d(0)).div(sumConfidence).rotateBy(gyroRotation);
            // If we only have one tracked target, take just the x and y values, divide by sumConfidence, and then rotate to the gyroRotation
        }
        return Optional.ofNullable(averagePose);
    }

    /**
     * Get the timestamp of the pose capture.
     * @return The latest timestamp.
     */
    public double getTimestamp() {
        return inputs.estimatedRearPoseTimestamp > inputs.estimatedFrontPoseTimestamp ? inputs.estimatedRearPoseTimestamp : inputs.estimatedFrontPoseTimestamp;
    }

    /**
     * Get the tracked targets from the front camera.
     * @return The tracked targets.
     */

    public List<PhotonTrackedTarget> getFrontTargets() {
        return io.getFrontTrackedTargets();
    }

    /**
     * Get the tracked targets from the rear camera.
     * @return The tracked targets.
     */
    public List<PhotonTrackedTarget> getRearTargets() {
        return io.getRearTrackedTargets();
    }

    /**
     * Get the fiducial IDs of the targets in the front camera.
     * @return A list of the IDs.
     */
    public int[] getFrontFiducialIDs() {
        return inputs.visibleFrontFiducialIDs;
    }

    /**
     * Get the fiducial IDs of the targets in the rear camera.
     * @return A list of the IDs.
     */
    public int[] getRearFiducialIDs() {
        return inputs.visibleRearFiducialIDs;
    }

    public void periodic() {
        Logger.processInputs("PhotonVision", inputs);
    }
}
