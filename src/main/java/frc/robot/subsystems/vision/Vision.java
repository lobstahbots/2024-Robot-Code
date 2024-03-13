package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private Pose2d robotPose = new Pose2d();
    
    public Vision(VisionIO io) {
       this.io = io;
    }

    /**
     * Get the estimated pose from both cameras.
     * @param odometryPose The current pose returned by the robot odometry to filter the vision estimate in comparison to.
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
        io.updateInputs(inputs, new Pose3d(robotPose));
        Logger.processInputs("PhotonVision", inputs);
    }

    public void update(Pose2d robotPose) {
        this.robotPose = robotPose;
    }
}
