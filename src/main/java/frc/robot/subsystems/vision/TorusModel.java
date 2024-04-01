// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.DoubleStream;

import org.photonvision.estimation.TargetModel;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class TorusModel extends TargetModel {
    public TorusModel(double radius, double thicknessRadius, double circleIntervalRad, double thicknessIntervalRad) {
        super(DoubleStream.iterate(0, val -> val <= 2 * Math.PI - circleIntervalRad, val -> val + circleIntervalRad)
                .mapToObj(angle -> {
                    Translation3d basePoint = new Translation3d(Math.cos(angle), Math.sin(angle), 0).times(radius);
                    List<Translation3d> points = new ArrayList<>();
                    for (double secondaryAngle = 0; secondaryAngle < 2 * Math.PI
                            - thicknessIntervalRad; secondaryAngle += thicknessIntervalRad) {
                        points.add(
                                basePoint.plus(new Translation3d(Math.cos(secondaryAngle), 0, Math.sin(secondaryAngle))
                                        .rotateBy(new Rotation3d(0, 0, angle))));
                    }
                    return points;
                }).flatMap(list -> list.stream()).collect(Collectors.toList()));
    }

    public TorusModel(double radius, double thicknessRadius) {
        this(radius, thicknessRadius, Math.PI / 12, Math.PI / 5);
    }
}
