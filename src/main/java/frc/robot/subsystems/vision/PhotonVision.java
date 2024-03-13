package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
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
     * @return The estimated pose.
     */
    public Poses getEstimatedPose(Pose2d odometryPose) {
        Pose2d resolvedFrontPose = null;
        Pose2d resolvedRearPose = null;
        Vector<N3> frontStdev = null;
        Vector<N3> rearStdev = null;

        Pose2d frontPose = inputs.estimatedFrontPose.toPose2d();
        double frontAmbiguity = Arrays.stream(inputs.frontAmbiguities).average().orElse(1);
        if (frontAmbiguity < (1 - VisionConstants.POSE_CONFIDENCE_FILTER_THRESHOLD) && frontPose.minus(odometryPose).getTranslation().getNorm() < VisionConstants.VISION_ODOMETRY_DIFFERENCE_FILTER_THRESHOLD) {
            resolvedFrontPose = frontPose;
            frontStdev = VisionConstants.BASE_STDEV.times(
                Math.pow(frontAmbiguity, VisionConstants.AMBIGUITY_TO_STDEV_EXP) // Start with ambiguity
                * Math.exp(1/inputs.visibleFrontFiducialIDs.length) * Math.pow(inputs.visibleFrontFiducialIDs.length, VisionConstants.APRIL_TAG_NUMBER_EXPONENT) // Multiply by the scaling for the number of AprilTags
                * Math.pow(inputs.frontTotalArea, 1 / VisionConstants.APRIL_TAG_AREA_CONFIDENCE_SCALE) * Math.log(2) / Math.log(inputs.frontTotalArea + 1) // Multiply by the scaling for the area of the AprilTags
            );
        }

        Pose2d rearPose = inputs.estimatedRearPose.toPose2d();
        double rearAmbiguity = Arrays.stream(inputs.rearAmbiguities).average().orElse(1);
        if (rearAmbiguity < (1 - VisionConstants.POSE_CONFIDENCE_FILTER_THRESHOLD) && rearPose.minus(odometryPose).getTranslation().getNorm() < VisionConstants.VISION_ODOMETRY_DIFFERENCE_FILTER_THRESHOLD) {
            resolvedRearPose = rearPose;
            rearStdev = VisionConstants.BASE_STDEV.times(
                Math.pow(rearAmbiguity, VisionConstants.AMBIGUITY_TO_STDEV_EXP) // Start with ambiguity
                * Math.exp(1/inputs.visibleRearFiducialIDs.length) * Math.pow(inputs.visibleRearFiducialIDs.length, VisionConstants.APRIL_TAG_NUMBER_EXPONENT) // Multiply by the scaling for the number of AprilTags
                * Math.pow(inputs.rearTotalArea, 1 / VisionConstants.APRIL_TAG_AREA_CONFIDENCE_SCALE) * Math.log(2) / Math.log(inputs.rearTotalArea + 1) // Multiply by the scaling for the area of the AprilTags
            );
        }

        return new Poses(Optional.ofNullable(resolvedFrontPose), Optional.ofNullable(resolvedRearPose), Optional.ofNullable(frontStdev), Optional.ofNullable(rearStdev));
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

    public record Poses(Optional<Pose2d> frontPose, Optional<Pose2d> rearPose, Optional<Vector<N3>> frontStdev, Optional<Vector<N3>> rearStdev) {};
}
