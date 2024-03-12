package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;

public class PhotonVisionReal implements PhotonVisionIO {
    private final PhotonCamera frontCamera;
    private final PhotonCamera rearCamera;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator rearPoseEstimator;
    private EstimatedRobotPose estimatedFrontPose = new EstimatedRobotPose(new Pose3d(), 0, new ArrayList<PhotonTrackedTarget>(), VisionConstants.POSE_STRATEGY);
    private EstimatedRobotPose estimatedRearPose = new EstimatedRobotPose(new Pose3d(), 0, new ArrayList<PhotonTrackedTarget>(), VisionConstants.POSE_STRATEGY);
    
    public PhotonVisionReal() {
        this.rearCamera = new PhotonCamera("photonvision_rear");
        this.frontCamera = new PhotonCamera("photonvision_front");
        this.frontPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY, frontCamera, VisionConstants.ROBOT_TO_FRONT_CAMERA);
        this.rearPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY, rearCamera, VisionConstants.ROBOT_TO_REAR_CAMERA);
    }

    public void updateInputs(PhotonVisionIOInputs inputs) {
        Optional<EstimatedRobotPose> frontPoseOptional = frontPoseEstimator.update();
        if (frontPoseOptional.isPresent()) {
            estimatedFrontPose = frontPoseOptional.get();
            PoseInformation frontPoseInformation = getPoseInformation(estimatedFrontPose);

            inputs.estimatedFrontPose = frontPoseInformation.estimatedPose;
            inputs.estimatedFrontPoseTimestamp = frontPoseInformation.timestamp;
            inputs.visibleFrontFiducialIDs = frontPoseInformation.fiducialIDs;
            inputs.frontConfidence = frontPoseInformation.confidence;
        }

        Optional<EstimatedRobotPose> rearPoseOptional = rearPoseEstimator.update();
        if (rearPoseOptional.isPresent()) {
            estimatedRearPose = rearPoseOptional.get();
            PoseInformation rearPoseInformation = getPoseInformation(estimatedRearPose);
            
            inputs.estimatedRearPose = rearPoseInformation.estimatedPose;
            inputs.estimatedRearPoseTimestamp = rearPoseInformation.timestamp;
            inputs.visibleRearFiducialIDs = rearPoseInformation.fiducialIDs;
            inputs.rearConfidence = rearPoseInformation.confidence;
        } 
    }

    public List<PhotonTrackedTarget> getFrontTrackedTargets() {
        return estimatedFrontPose.targetsUsed;
    }

    public List<PhotonTrackedTarget> getRearTrackedTargets() {
        return estimatedRearPose.targetsUsed;
    }

    @AutoLogOutput
    public Pose3d[] getFrontTagPoses() {
        var frontTargets = estimatedFrontPose.targetsUsed;
        Pose3d[] frontTagPoses = new Pose3d[frontTargets.size()];
        for(int i = 0; i < frontTagPoses.length; i++) {
            frontTagPoses[i] = aprilTagFieldLayout.getTagPose(frontTargets.get(i).getFiducialId()).get();
        }
        return frontTagPoses;
    }

    @AutoLogOutput
    public Pose3d[] getRearTagPoses() {
        var rearTargets = estimatedRearPose.targetsUsed;
        Pose3d[] rearTagPoses = new Pose3d[rearTargets.size()];
        for(int i = 0; i < rearTagPoses.length; i++) {
            rearTagPoses[i] = aprilTagFieldLayout.getTagPose(rearTargets.get(i).getFiducialId()).get();
        }
        return rearTagPoses;
    }

    private record PoseInformation(Pose3d estimatedPose, double timestamp, int[] fiducialIDs, double confidence) {}

    private static PoseInformation getPoseInformation(EstimatedRobotPose estimatedPose) {
        var targetsSeen = estimatedPose.targetsUsed.size();
        var visibleFiducialIDs = new int[targetsSeen];
        
        double area = 0;
        double ambiguitySum = 0;
       
        for (int i = 0; i < targetsSeen; i++) {
            var target = estimatedPose.targetsUsed.get(i);
            visibleFiducialIDs[i] = target.getFiducialId();
            ambiguitySum += target.getPoseAmbiguity();
            area += target.getArea() / 100; // Area is returned in percent but we want fraction
            // See https://docs.photonvision.org/en/latest/docs/programming/photonlib/getting-target-data.html#getting-data-from-a-target
        }  

        int count = visibleFiducialIDs.length;
        double confidence = (1 - (ambiguitySum / count)) // Get the average ambiguity and turn it into confidence
            * Math.exp(-1/count) * Math.pow(count, VisionConstants.APRIL_TAG_NUMBER_EXPONENT) // Multiply by the confidence scaling for the number of AprilTags
            * Math.log(area+1) / (Math.pow(area, 1 / VisionConstants.APRIL_TAG_AREA_CONFIDENCE_SCALE) * Math.log(2)); // Multiply by the confidence scaling for the area of the AprilTags
        // For more information about the scaling see https://www.desmos.com/calculator/hw9b2s1mlw

        return new PoseInformation(estimatedPose.estimatedPose, estimatedPose.timestampSeconds, visibleFiducialIDs, confidence);
    }
}
