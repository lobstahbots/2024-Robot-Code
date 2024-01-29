package frc.robot.subsystems;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface PhotonVisionIO {
    @AutoLog
    public static class PhotonVisionIOInputs {
        public EstimatedRobotPose estimatedFrontPose = new EstimatedRobotPose(
            null, 0, null, null);
        public EstimatedRobotPose estimatedRearPose = new EstimatedRobotPose(
            null, 0, null, null);
    }

    /**
     * Get the estimated pose from the front camera.
     * @return The estimated pose.
     */
    public EstimatedRobotPose getEstimatedFrontPose();
    
    /**
     * Get the estimated pose from the rear camera.
     * @return The estimated pose.
     */
    public EstimatedRobotPose getEstimatedRearPose();

    /**
     * Get the tracked targets from the front camera.
     * @return The tracked targets.
     */
    public List<PhotonTrackedTarget> getFrontTargets();

    /**
     * Get the tracked targets from the rear camera.
     * @return The tracked targets.
     */
    public List<PhotonTrackedTarget> getRearTargets();

    /**
     * Get the fiducial IDs of the targets in the front camera.
     * @return A list of the IDs.
     */
    public List<Integer> getFrontFiducialIDs();

    /**
     * Get the fiducial IDs of the targets in the rear camera.
     * @return A list of the IDs.
     */
    public List<Integer> getRearFiducialIDs();

    public default void updateInputs(PhotonVisionIOInputs inputs) {}
}
