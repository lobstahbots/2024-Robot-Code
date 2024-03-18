package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public Pose3d estimatedFrontPose = new Pose3d();
        public Pose3d estimatedRearPose = new Pose3d();
        public double estimatedFrontPoseTimestamp = 0.0;
        public double estimatedRearPoseTimestamp = 0.0;
        public int[] visibleFrontFiducialIDs = new int[] {};
        public int[] visibleRearFiducialIDs = new int[] {};
        public double[] frontAmbiguities = new double[] {};
        public double[] rearAmbiguities = new double[] {};
        public double frontTotalArea = 0.0;
        public double rearTotalArea = 0.0;
    }

    public default List<PhotonTrackedTarget> getFrontTrackedTargets() {
        return new ArrayList<>();
    }

    public default List<PhotonTrackedTarget> getRearTrackedTargets() {
        return new ArrayList<>();
    }

    public default void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {}

    public default void update(Pose2d robotPose) {}

    public default void periodic() {}
}
