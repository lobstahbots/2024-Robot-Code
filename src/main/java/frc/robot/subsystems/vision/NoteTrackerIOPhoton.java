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

/** Add your docs here. */
public class NoteTrackerIOPhoton implements NoteTrackerIO {
    private final PhotonCamera noteCamera;
    private final Alert cameraDisconnectedAlert;
    private List<PhotonTrackedTarget> notes = new ArrayList<>();

    public NoteTrackerIOPhoton() {
        noteCamera = new PhotonCamera("NoteCam");
        cameraDisconnectedAlert = new Alert("Note camera has disconnected.", AlertType.ERROR, () -> !noteCamera.isConnected());
    }

    public List<PhotonTrackedTarget> getNotes() {
        return notes;
    }

    public void updateInputs(NoteTrackerIOInputs inputs, Pose2d robotPose) {
        var result = noteCamera.getLatestResult();
        NoteTrackerIO.updateInputs(inputs, result.targets);
    }
}
