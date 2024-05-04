// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public interface NoteTrackerIO {
    @AutoLog
    public static class NoteTrackerIOInputs {
        public double[] yaws = new double[] {};
        public double[] pitches = new double[] {};
        public double[] areas = new double[] {};
        public double[] skews = new double[] {};
        public double[] minXs = new double[] {};
        public double[] minYs = new double[] {};
        public double[] maxXs = new double[] {};
        public double[] maxYs = new double[] {};
    }

    public default void updateInputs(NoteTrackerIOInputs inputs, Pose2d robotPose) {};

    public default void periodic() {};
}
