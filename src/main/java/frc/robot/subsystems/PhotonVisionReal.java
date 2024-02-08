package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants.VisionConstants;

public class PhotonVisionReal implements PhotonVisionIO {
    private final PhotonCamera frontCamera;
    private final PhotonCamera rearCamera;
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator rearPoseEstimator;
    private EstimatedRobotPose estimatedFrontPose = new EstimatedRobotPose(null, 0, getFrontTrackedTargets(), null);
    private EstimatedRobotPose estimatedRearPose = new EstimatedRobotPose(null, 0, getFrontTrackedTargets(), null);
    
    public PhotonVisionReal() {
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        this.rearCamera = new PhotonCamera("photonvision_rear");
        this.frontCamera = new PhotonCamera("photonvision_front");
        this.frontPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY, frontCamera, VisionConstants.ROBOT_TO_FRONT_CAMERA);
        this.rearPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY, rearCamera, VisionConstants.ROBOT_TO_REAR_CAMERA);
    }

    public void updateInputs(PhotonVisionIOInputs inputs) {
        double frontAmbiguitySum = 0;
        double rearAmbiguitySum = 0;

        inputs.rearConfidence = 0;
        inputs.frontConfidence = 0;

        Optional<EstimatedRobotPose> frontPoseOptional = frontPoseEstimator.update();
        if (frontPoseOptional.isPresent()) {
            estimatedFrontPose = frontPoseOptional.get();

            inputs.estimatedFrontPose = estimatedFrontPose.estimatedPose;
            inputs.estimatedFrontPoseTimestamp = estimatedFrontPose.timestampSeconds;

            var frontTargetsSeen = estimatedFrontPose.targetsUsed.size();
            inputs.visibleFrontFiducialIDs = new int[frontTargetsSeen];

            for (int i = 0; i < frontTargetsSeen; i++) {
                var target = estimatedFrontPose.targetsUsed.get(i);
                inputs.visibleFrontFiducialIDs[i] = target.getFiducialId();
                frontAmbiguitySum += target.getPoseAmbiguity();
            }   

            inputs.frontConfidence = frontAmbiguitySum / inputs.visibleFrontFiducialIDs.length;
        }
        Optional<EstimatedRobotPose> rearPoseOptional = rearPoseEstimator.update();
        
        if (rearPoseOptional.isPresent()) {
            estimatedRearPose = rearPoseOptional.get();

            inputs.estimatedRearPose = estimatedRearPose.estimatedPose;
            inputs.estimatedRearPoseTimestamp = estimatedRearPose.timestampSeconds;
        
            var rearTargetsSeen = estimatedRearPose.targetsUsed.size();
            inputs.visibleRearFiducialIDs = new int[rearTargetsSeen];
        
       
            for (int i = 0; i < rearTargetsSeen; i++) {
                var target = estimatedRearPose.targetsUsed.get(i);
                inputs.visibleFrontFiducialIDs[i] = target.getFiducialId();
                rearAmbiguitySum += target.getPoseAmbiguity();
            }  

            inputs.rearConfidence = rearAmbiguitySum / inputs.visibleRearFiducialIDs.length; 
        } 
    }

    @AutoLogOutput
    public List<PhotonTrackedTarget> getFrontTrackedTargets() {
        return estimatedFrontPose.targetsUsed;
    }

    @AutoLogOutput
    public List<PhotonTrackedTarget> getRearTrackedTargets() {
        return estimatedRearPose.targetsUsed;
    }
}
