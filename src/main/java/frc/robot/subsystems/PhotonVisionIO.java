package frc.robot.subsystems;

import java.util.ArrayList;
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
        public List<PhotonTrackedTarget> frontTargets = new ArrayList<>();
        public List<PhotonTrackedTarget> rearTargets = new ArrayList<>();
        public List<Integer> visibleFrontFiducialIDs = new ArrayList<>();
        public List<Integer> visibleRearFiducialIDs = new ArrayList<>();  
    }

    public default void updateInputs(PhotonVisionIOInputs inputs) {}
}
