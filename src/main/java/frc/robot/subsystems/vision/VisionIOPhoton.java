package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants.VisionConstants;
import frc.robot.networkalerts.Alert;
import frc.robot.networkalerts.Alert.AlertType;

public class VisionIOPhoton implements VisionIO {
    private final PhotonCamera frontCamera;
    private final PhotonCamera rearCamera;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator rearPoseEstimator;
    private EstimatedRobotPose estimatedFrontPose = new EstimatedRobotPose(null, null, 0.0, 0.0, 0.0, 0.0,
            new ArrayList<PhotonTrackedTarget>(), VisionConstants.POSE_STRATEGY);
    private EstimatedRobotPose estimatedRearPose = new EstimatedRobotPose(null, null, 0.0, 0.0, 0.0, 0.0,
            new ArrayList<PhotonTrackedTarget>(), VisionConstants.POSE_STRATEGY);
    private final Alert frontDisconnectedAlert;
    private final Alert rearDisconnectedAlert;

    public VisionIOPhoton() {
        this.rearCamera = new PhotonCamera("photonvision4");
        this.frontCamera = new PhotonCamera("photonvision2");
        this.frontPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY,
                frontCamera, VisionConstants.ROBOT_TO_FRONT_CAMERA);
        this.rearPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, VisionConstants.POSE_STRATEGY, rearCamera,
                VisionConstants.ROBOT_TO_REAR_CAMERA);
        frontDisconnectedAlert = new Alert("Front camera has disconnected.", AlertType.ERROR, () -> !frontCamera.isConnected());
        rearDisconnectedAlert = new Alert("Rear camera has disconnected.", AlertType.ERROR, () -> !rearCamera.isConnected());
    }

    public void updateInputs(VisionIOInputs inputs, Pose3d robotPoseMeters) {
        Optional<EstimatedRobotPose> frontPoseOptional = frontPoseEstimator.update();
        if (frontPoseOptional.isPresent()) {
            estimatedFrontPose = frontPoseOptional.get();

            inputs.bestEstimatedFrontPose = estimatedFrontPose.bestEstimatedPose;
            inputs.altEstimatedFrontPose = estimatedFrontPose.alternateEstimatedPose;
            inputs.bestFrontReprojErr = estimatedFrontPose.bestReprojError;
            inputs.altFrontReprojErr = estimatedFrontPose.altReprojError;
            inputs.estimatedFrontPoseTimestamp = estimatedFrontPose.timestampSeconds;
            inputs.visibleFrontFiducialIDs = estimatedFrontPose.fiducialIDsUsed;
            inputs.frontTotalArea = estimatedFrontPose.totalArea;
            inputs.frontAmbiguity = estimatedFrontPose.multiTagAmbiguity;
        } else {
            inputs.bestEstimatedFrontPose = null;
            inputs.altEstimatedFrontPose = null;
            inputs.bestFrontReprojErr = 0.0;
            inputs.visibleFrontFiducialIDs = new int[0];
            inputs.frontTotalArea = 0;
            inputs.frontAmbiguity = 0;
            inputs.altFrontReprojErr = 0;
        }

        Optional<EstimatedRobotPose> rearPoseOptional = rearPoseEstimator.update();
        if (rearPoseOptional.isPresent()) {
            estimatedRearPose = rearPoseOptional.get();

            inputs.bestEstimatedRearPose = estimatedRearPose.bestEstimatedPose;
            inputs.altEstimatedRearPose = estimatedRearPose.alternateEstimatedPose;
            inputs.bestRearReprojErr = estimatedRearPose.bestReprojError;
            inputs.altRearReprojErr = estimatedRearPose.altReprojError;
            inputs.estimatedRearPoseTimestamp = estimatedRearPose.timestampSeconds;
            inputs.visibleRearFiducialIDs = estimatedRearPose.fiducialIDsUsed;
            inputs.rearTotalArea = estimatedRearPose.totalArea;
            inputs.rearAmbiguity = estimatedRearPose.multiTagAmbiguity;
        } else {
            inputs.bestEstimatedRearPose = null;
            inputs.altEstimatedRearPose = null;
            inputs.bestRearReprojErr = 0.0;
            inputs.visibleRearFiducialIDs = new int[0];
            inputs.rearTotalArea = 0;
            inputs.rearAmbiguity = 0;
            inputs.altRearReprojErr = 0;
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
        for (int i = 0; i < frontTagPoses.length; i++) {
            frontTagPoses[i] = aprilTagFieldLayout.getTagPose(frontTargets.get(i).getFiducialId()).get();
        }
        return frontTagPoses;
    }

    @AutoLogOutput
    public Pose3d[] getRearTagPoses() {
        var rearTargets = estimatedRearPose.targetsUsed;
        Pose3d[] rearTagPoses = new Pose3d[rearTargets.size()];
        for (int i = 0; i < rearTagPoses.length; i++) {
            rearTagPoses[i] = aprilTagFieldLayout.getTagPose(rearTargets.get(i).getFiducialId()).get();
        }
        return rearTagPoses;
    }
}
