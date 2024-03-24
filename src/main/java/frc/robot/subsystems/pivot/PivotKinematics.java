package frc.robot.subsystems.pivot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.PivotConstants;
import stl.trajectory.AlliancePoseMirror;

public class PivotKinematics {
    @AutoLogOutput
    public static DoubleSupplier getShotAngle(Supplier<Pose2d> targetSupplier, Supplier<Pose2d> poseSupplier) {
        Pose2d targetPose = AlliancePoseMirror.mirrorPose2d(targetSupplier.get());
        return () -> PivotConstants.shotAngleMap.get(Math.abs(targetPose.minus(poseSupplier.get()).getTranslation().getNorm()));
    }
}