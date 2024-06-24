// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import stl.math.LobstahMath;

/** Add your docs here. */
public class NoteTrackerIOSim implements NoteTrackerIO {
    private final PhotonCamera noteCamera;
    private final SimCameraProperties noteCameraProp;
    private final PhotonCameraSim noteCameraSim;
    private final VisionSystemSim visionSystemSim;
    private final List<VisionTargetSim> noteTargets;
    private final TimeInterpolatableBuffer<Pose2d> robotPoseBuffer = TimeInterpolatableBuffer.createBuffer(1.5);
    private List<PhotonTrackedTarget> notes = new ArrayList<>();

    public NoteTrackerIOSim() {
        visionSystemSim = new VisionSystemSim("notes");
        noteCamera = new PhotonCamera("photonvision_notes");
        noteCameraProp = new SimCameraProperties();
        noteCameraProp.setAvgLatencyMs(VisionConstants.NOTE_CAMERA_AVG_LATENCY_MS);
        noteCameraProp.setLatencyStdDevMs(VisionConstants.NOTE_CAMERA_LATENCY_STDEV_MS);
        noteCameraProp.setCalibError(VisionConstants.NOTE_AVG_ERROR_PX, VisionConstants.NOTE_ERROR_STDEV_PX);
        noteCameraProp.setCalibration(VisionConstants.NOTE_CAMERA_RES_WIDTH, VisionConstants.NOTE_CAMERA_RES_HEIGHT,
                Rotation2d.fromDegrees(VisionConstants.NOTE_CAMERA_FOV_DEG));
        noteCameraProp.setFPS(VisionConstants.FPS);
        noteCameraSim = new PhotonCameraSim(noteCamera, noteCameraProp);
        TargetModel noteModel = new TorusModel(FieldConstants.NOTE_RADIUS, FieldConstants.NOTE_THICKNESS_RADIUS);
        noteTargets = Stream.of(FieldConstants.NOTES_SIM_POSES)
                .map((Pose3d pose) -> pose
                        .transformBy(new Transform3d(0, 0, FieldConstants.NOTE_THICKNESS_RADIUS, new Rotation3d())))
                .map(pose -> new VisionTargetSim(pose, noteModel)).collect(Collectors.toList());
        visionSystemSim.addVisionTargets(noteTargets.toArray(VisionTargetSim[]::new));
        visionSystemSim.addCamera(noteCameraSim, VisionConstants.ROBOT_TO_NOTE_CAMERA);
    }

    private void updateNoteTargets() {
        for (int i = 0; i < noteTargets.size(); i++) {
            noteTargets.get(i).setPose(FieldConstants.NOTES_SIM_POSES[i]);
        }
    }

    public void updateInputs(NoteTrackerIOInputs inputs, Pose2d robotPose) {
        robotPoseBuffer.addSample(Timer.getFPGATimestamp(), robotPose);
        var optTimestamp = noteCameraSim.consumeNextEntryTime();
        if (optTimestamp.isEmpty()) return;

        // when this result "was" read by NT
        long timestampNT = optTimestamp.get();
        // this result's processing latency in milliseconds
        double latencyMillis = noteCameraSim.prop.estLatencyMs();
        // the image capture timestamp in seconds of this result
        double timestampCapture = timestampNT / 1e6 - latencyMillis / 1e3;

        // use camera pose from the image capture timestamp
        Pose2d lateRobotPose = robotPoseBuffer.getSample(timestampCapture).orElse(new Pose2d());
        Pose3d lateCameraPose = new Pose3d(lateRobotPose).plus(VisionConstants.ROBOT_TO_NOTE_CAMERA);

        // process a PhotonPipelineResult with visible targets
        var camResult = noteCameraSim.process(latencyMillis, lateCameraPose, noteTargets);
        camResult.setTimestampSeconds(timestampCapture);

        // publish this info to NT at estimated timestamp of receive
        noteCameraSim.submitProcessedFrame(camResult, timestampNT);

        notes = camResult.getTargets();

        inputs.areas = notes.stream().mapToDouble(note -> note.getArea()).toArray();
        inputs.pitches = notes.stream().mapToDouble(note -> note.getPitch()).toArray();
        inputs.skews = notes.stream().mapToDouble(note -> note.getSkew()).toArray();
        inputs.yaws = notes.stream().mapToDouble(note -> note.getYaw()).toArray();
        inputs.minXs = new double[notes.size()];
        inputs.minYs = new double[notes.size()];
        inputs.maxXs = new double[notes.size()];
        inputs.maxYs = new double[notes.size()];
        for (int i = 0; i < notes.size(); i++) {
            var note = notes.get(i);
            var boundaries = LobstahMath.getBoundaries(note.getMinAreaRectCorners());
            inputs.minXs[i] = boundaries.minX();
            inputs.minYs[i] = boundaries.minY();
            inputs.maxXs[i] = boundaries.maxX();
            inputs.maxYs[i] = boundaries.maxY();
        }

        Logger.recordOutput("RealNoteOffsets", noteTargets.stream()
                .map(noteTarget -> noteTarget.getPose().toPose2d().minus(robotPose)).toArray(Transform2d[]::new));
    }

    public void periodic() {
        updateNoteTargets();
    }
}
