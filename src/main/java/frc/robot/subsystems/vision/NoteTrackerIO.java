// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public interface NoteTrackerIO {
    @AutoLog
    public static class NoteTrackerIOInputs {
        public PhotonTrackedTarget[] notes; // One might think that this is a bad idea, but turns out
                                            // that PhotonTrackedTargets work in AdvantageKit
                                            // So this is nice
    }

    public default List<PhotonTrackedTarget> getNotes() {
        return new ArrayList<>();
    }

    public default void update(Pose2d robotPose) {}

    public default void updateInputs(NoteTrackerIOInputs inputs) {};

    public default void periodic() {};
}
