// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.networkalerts.Alert;
import frc.robot.networkalerts.Alert.AlertType;
import stl.math.LobstahMath;

/** Add your docs here. */
public class NoteTrackerIOPhoton implements NoteTrackerIO {
    private final PhotonCamera noteCamera;
    private final Alert cameraDisconnectedAlert;
    private List<PhotonTrackedTarget> notes = new ArrayList<>();

    public NoteTrackerIOPhoton() {
        noteCamera = new PhotonCamera("NoteCam");
        cameraDisconnectedAlert = new Alert("Note camera has disconnected.", AlertType.ERROR,
                () -> !noteCamera.isConnected());
    }

    public void updateInputs(NoteTrackerIOInputs inputs, Pose2d robotPose) {
        notes = noteCamera.getLatestResult().getTargets();
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
    }
}
