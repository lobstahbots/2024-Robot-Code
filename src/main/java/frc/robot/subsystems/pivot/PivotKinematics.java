package frc.robot.subsystems.pivot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import stl.trajectory.AlliancePoseMirror;

public class PivotKinematics {
    private static final InterpolatingDoubleTreeMap shotAngleMap = new InterpolatingDoubleTreeMap();

    public static void setAngles() {
        shotAngleMap.put(1.039, Units.degreesToRadians(90.0));
        shotAngleMap.put(24.0,Units.degreesToRadians(19.0));
    }

    @AutoLogOutput
    public static DoubleSupplier getShotAngle(Supplier<Pose2d> targetSupplier, Supplier<Pose2d> poseSupplier) {
        Pose2d targetPose = AlliancePoseMirror.mirrorPose2d(targetSupplier.get());
        return () -> shotAngleMap.get(Math.abs(targetPose.minus(poseSupplier.get()).getTranslation().getNorm()));
    }
}