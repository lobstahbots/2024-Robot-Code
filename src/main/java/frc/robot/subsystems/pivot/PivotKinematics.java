package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.PivotConstants;

public class PivotKinematics {
    public static Translation2d angleToSimPivotTransform(double pivotAngleRadians) {
        pivotAngleRadians = Math.abs(pivotAngleRadians);
        pivotAngleRadians %= Math.PI/2;
        return new Translation2d(Units.inchesToMeters(PivotConstants.PIVOT_SIM_ROTATION_POINT_DISTANCE_INCHES)*Math.asin(pivotAngleRadians), -Units.inchesToMeters(PivotConstants.PIVOT_SIM_ROTATION_POINT_DISTANCE_INCHES)*Math.acos(pivotAngleRadians));
    }
}