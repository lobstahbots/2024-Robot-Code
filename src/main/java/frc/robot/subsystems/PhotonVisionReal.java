package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

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
    
    public PhotonVisionReal() {
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        this.rearCamera = new PhotonCamera("photonvision_rear");
        this.frontCamera = new PhotonCamera("photonvision_front");
        this.frontPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY, frontCamera, VisionConstants.ROBOT_TO_FRONT_CAMERA);
        this.rearPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY, rearCamera, VisionConstants.ROBOT_TO_REAR_CAMERA);
    }

    public void updateInputs(PhotonVisionIOInputs inputs) {
        Optional<EstimatedRobotPose> frontPoseOptional = frontPoseEstimator.update();
        if (frontPoseOptional.isPresent()) inputs.estimatedFrontPose = frontPoseOptional.get();
        Optional<EstimatedRobotPose> rearPoseOptional = rearPoseEstimator.update();
        if (rearPoseOptional.isPresent()) inputs.estimatedRearPose = rearPoseOptional.get();
        inputs.frontTargets = inputs.estimatedFrontPose.targetsUsed;
        inputs.rearTargets = inputs.estimatedRearPose.targetsUsed;
        inputs.visibleFrontFiducialIDs = new ArrayList<>();
        inputs.visibleRearFiducialIDs = new ArrayList<>();
        double frontAmbiguitySum = 0;
        double rearAmbiguitySum = 0;
        for (PhotonTrackedTarget target : inputs.frontTargets) {
            inputs.visibleFrontFiducialIDs.add(target.getFiducialId());
            frontAmbiguitySum += target.getPoseAmbiguity();
        }   
        for (PhotonTrackedTarget target : inputs.rearTargets) {
            inputs.visibleRearFiducialIDs.add(target.getFiducialId());
            rearAmbiguitySum += target.getPoseAmbiguity();
        }
        inputs.rearConfidence = rearAmbiguitySum / inputs.rearTargets.size();  
        inputs.frontConfidence = frontAmbiguitySum / inputs.frontTargets.size();
    }
}
