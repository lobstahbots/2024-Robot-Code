// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public interface NoteTrackerIO {
    @AutoLog
    public static class NoteTrackerIOInputs {
        public double[] yaws = new double[] {};
        public double[] pitches = new double[] {};
        public double[] areas = new double[] {};
        public double[] skews = new double[] {};
        public Transform3d[] cameraToTargets = new Transform3d[] {};
    }

    public default void updateInputs(NoteTrackerIOInputs inputs, Pose2d robotPose) {};

    public static void updateInputs(NoteTrackerIOInputs inputs, List<PhotonTrackedTarget> targets) {
        int len = targets.size();
        inputs.yaws = new double[len];
        inputs.pitches = new double[len];
        inputs.areas = new double[len];
        inputs.skews = new double[len];
        inputs.cameraToTargets = new Transform3d[len];
        for (int i = 0; i < len; i++) {
            var target = targets.get(i);
            inputs.yaws[i] = target.getYaw();
            inputs.pitches[i] = target.getPitch();
            inputs.areas[i] = target.getArea();
            inputs.skews[i] = target.getSkew();
            inputs.cameraToTargets[i] = target.getBestCameraToTarget();
        }
    }

    public default void periodic() {};
}
