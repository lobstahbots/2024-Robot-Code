// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.vision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.drive.SwerveKinematicLimits;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class PathConstants {
    public static final Pose2d TARGET_POSE = new Pose2d(16, 7, Rotation2d.fromDegrees(180));
    public static final Pose2d INITIAL_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final PathConstraints CONSTRAINTS = new PathConstraints(
        1, 1,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static final Pose2d STATION_1 = new Pose2d(0.4119143784046173, 7.161474227905273,
        Rotation2d.fromRotations(0));
    public static final Pose2d STATION_2 = new Pose2d(0.5068893432617188, 3.710716009140014,
        Rotation2d.fromRotations(0));
    public static final Pose2d STATION_3 = new Pose2d(0.44357267022132874, 2.3525, Rotation2d.fromRotations(0));
    public static final double SPIN_UP_FLYWHEELS_RADIUS_METERS = 7.62;
    public static final double AMP_ALIGN_DEADBAND = Units.inchesToMeters(6);
    public static final double SOURCE_ALIGN_DEADBAND = Units.inchesToMeters(6);
  }

  public static class IOConstants {
    public static final double JOYSTICK_DEADBAND = 0.1;
    public static class DriverIOConstants {
      public static final int DRIVER_CONTROLLER_PORT = 0;
      public static final int STRAFE_X_AXIS = 0;
      public static final int STRAFE_Y_AXIS = 1;
      public static final int ROTATION_AXIS = 2;
      public static final int ALIGN_TO_AMP_BUTTON_ID = 4;
      public static final int ALIGN_TO_SOURCE_BUTTON_ID = 2;
      public static final int ALIGN_TO_SPEAKER_BUTTON_ID = 3;
      public static final int INTAKE_BUTTON_ID = 5;
      public static final int INDEXER_BUTTON_ID = 7;
    }
    public static class OperatorIOConstants {
      public static final int OPERATOR_CONTROLLER_PORT = 1;
      public static final int INDEXER_BUTTON_ID = 1;
      public static final int SHOOTER_BUTTON_ID = 6;
      public static final int UNSHOOTER_BUTTON_ID = 5;
      public static final int OUTTAKE_BUTTON_ID = 3;
      public static final int AMP_BUTTON_ID = 4;
      public static final int PIVOT_ANGLE_AXIS = 1;
      public static final int SOURCE_BUTTON_ID = 2;
      public static final int SUBWOOFER_POV_ANGLE = 0;
      public static final int WING_POV_ANGLE = 90;
      public static final int PODIUM_POV_ANGLE = 270;
      public static final int PASS_POV_ANGLE = 180;
      public static final int USER_SIGNAL_BUTTON_ID = 7;
      public static final int SUBWOOFER_BACKSHOT_ID = 9;
      public static final int PODIUM_BACKSHOT_ID = 10;
    }
  }

  public static class RobotConstants {
    public static final double WHEELBASE = Units.inchesToMeters(28);
    public static final double TRACK_WIDTH = Units.inchesToMeters(28);
    public static final double RADIUS = Units.inchesToMeters(new Translation2d(20, 20).getNorm());
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
    public static final double DRIVE_GEAR_RATIO = 4.71;
    public static final double ANGLE_GEAR_RATIO = 6.1;
    public static final double CLIMBER_SPEED = 1.0;
  }

  public static class DriveConstants {
    public static final double MAX_ACCELERATION = 3;
    public static final double MAX_DRIVE_SPEED = 10;
    public static final double MAX_ANGULAR_SPEED = MAX_DRIVE_SPEED / RobotConstants.RADIUS;
    public static final double SLOWDOWN_PERCENT = 0.5;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
    public static final int ANGLE_MOTOR_CURRENT_LIMIT = 40;
    public static final Translation2d[] MODULE_LOCATIONS = new Translation2d[] {
        new Translation2d(RobotConstants.WHEELBASE / 2.0, RobotConstants.TRACK_WIDTH / 2.0),
        new Translation2d(RobotConstants.WHEELBASE / 2.0, -RobotConstants.TRACK_WIDTH / 2.0),
        new Translation2d(-RobotConstants.WHEELBASE / 2.0, -RobotConstants.TRACK_WIDTH / 2.0),
        new Translation2d(-RobotConstants.WHEELBASE / 2.0, RobotConstants.TRACK_WIDTH / 2.0)
    };
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        MODULE_LOCATIONS[0], MODULE_LOCATIONS[1], MODULE_LOCATIONS[2], MODULE_LOCATIONS[3]);
    public static final SwerveKinematicLimits MODULE_LIMITS = new SwerveKinematicLimits(MAX_DRIVE_SPEED,
        MAX_ACCELERATION, MAX_ANGULAR_SPEED);

    public static boolean FIELD_CENTRIC = true;
    public static final boolean IS_OPEN_LOOP = false;

    public static final double PATH_MAX_ACCEL = 3;
    public static final double PATH_MAX_VELOCITY = 3;

    public static final double TURN_KP = 2;
    public static final double TURN_KI = 0;
    public static final double TURN_KD = 0;

    public static final double TURN_DEADBAND = Units.degreesToRadians(5);

    public static class FrontLeftModuleConstants {
      public static final int moduleID = 0;
      public static final int driveID = 12;
      public static final int angleID = 13;
      public static final double angleOffset = -90;
      public static final boolean inverted = true;
    }

    public static class BackLeftModuleConstants {
      public static final int moduleID = 2;
      public static final int driveID = 16;
      public static final int angleID = 17;
      public static final double angleOffset = 180;
      public static final boolean inverted = true;
    }

    public static class FrontRightModuleConstants {
      public static final int moduleID = 1;
      public static final int driveID = 14;
      public static final int angleID = 15;
      public static final double angleOffset = 0;
      public static final boolean inverted = true;
    }

    public static class BackRightModuleConstants {
      public static final int moduleID = 3;
      public static final int driveID = 10;
      public static final int angleID = 11;
      public static final double angleOffset = 90;
      public static final boolean inverted = true;
    }
  }

  public static class SwerveConstants {
    public static final boolean invertGyro = true;

    public static final double KS = 0.1;
    public static final double KA = 0.1;
    public static final double KV = 0.1;

    public static final double DRIVING_ENCODER_POSITION_CONVERSION_FACTOR = 1;
    public static final double DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR = DRIVING_ENCODER_POSITION_CONVERSION_FACTOR
        / 60.0;
    public static final double TURNING_ENCODER_POSITION_CONVERSION_FACTOR = (2 * Math.PI);
    public static final double TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR = TURNING_ENCODER_POSITION_CONVERSION_FACTOR
        / 60.0;

    public static final double TURN_PID_MIN_INPUT = -Math.PI;
    public static final double TURN_PID_MAX_INPUT = Math.PI;

    public static final double DRIVE_PID_MIN_OUTPUT = -1;
    public static final double DRIVE_PID_MAX_OUTPUT = 1;
    public static final double DRIVE_PID_P = 0.1;
    public static final double DRIVE_PID_I = 0;
    public static final double DRIVE_PID_D = 0;
    public static final double DRIVE_PID_FF = 0;

    public static final double TURN_PID_MIN_OUTPUT = -2 * Math.PI;
    public static final double TURN_PID_MAX_OUTPUT = 2 * Math.PI;
    public static final double TURN_PID_P = 2;
    public static final double TURN_PID_I = 0;
    public static final double TURN_PID_D = 0;
    public static final double TURN_PID_FF = 0;

    public static final double AIM_VELOCITY_COMPENSATION_DEADBAND = 0.3;
  }

  public static class IntakeConstants {
    public static final double INTAKE_SPEED = 1;
    public static final double OUTTAKE_SPEED = -0.5;
    public static final int INTAKE_MOTOR_ID = 33;
    public static final int INDEXER_MOTOR_ID = 44;
    public static final double CURRENT_LIMIT = 40;
  }

  public static class ShooterConstants {
    public static final double SHOOTER_SPEED = -1;
    public static final double UNSHOOTER_SPEED = 0.5;
    public static final double SPIN_UP_SPEED = 0;
    public static final double PASS_SPEED = -0.6;
    public static final double AMP_SPEED = -0.175;
    public static final int UPPER_SHOOTER_ID = 5;
    public static final int LOWER_SHOOTER_ID = 32;
    public static final double CURRENT_LIMIT = 40;
    public static final double SHOOT_TIME = 2; // in seconds
    public static final double SHOOTING_FLYWHEEL_VELOCITY_DEADBAND_FACTOR = 0.9;

    public static final double PID_P = 0.05;
    public static final double PID_I = 0;
    public static final double PID_D = 0;

    public static final double KS = 0;
    public static final double KV = 0;
    public static final double KA = 0;

    public static final double MAX_VELOCITY = 100;
    public static final double MAX_ACCELERATION = 100;
    public static final double PID_TOLERANCE = 1;
  }

  public static class SimConstants {
    public static final double LOOP_TIME = 0.02;
    public static final boolean REPLAY = false;
    public static final String REPLAY_LOG_PATH = "Log_24-03-10_09-23-09_q61.wpilog";
  }

  public static class PivotConstants {
    public static final double PID_P = 0.25;
    public static final double PID_I = 0;
    public static final double PID_D = 0;

    public static final double KS = 0.56453;
    public static final double KG = 0.38989;
    public static final double KV = 0.0015868;
    public static final double KA = 0.0027206;

    public static final double MAX_VELOCITY = 0;
    public static final double MAX_ACCELERATION = 0;
    public static final double PID_TOLERANCE = 1;
    public static final int MAX_CURRENT = 30;

    public static final double PIVOT_SIM_ROTATION_POINT_DISTANCE_INCHES = 5;
    public static final double ORIGIN_TO_TOWER_MOUNT_X_DIST = Units.inchesToMeters(16.5);
    public static final double ORIGIN_TO_ARM_MOUNT_X_DIST = Units.inchesToMeters(13.5);
    public static final double ORIGIN_TO_TOWER_MOUNT_Y_DIST = Units.inchesToMeters(8.65);
    public static final double ORIGIN_TO_ARM_MOUNT_Z_OFFSET_DIST = Units.inchesToMeters(-2);
    public static final double ORIGIN_TO_ARM_MOUNT_Z_DIST = Units.inchesToMeters(12);
    public static final double ORIGIN_TO_ARM_MOUNT_Y_DIST = Units.inchesToMeters(6);
    public static final double ORIGIN_TO_TOWER_MOUNT_Z_DIST = Units.inchesToMeters(3.375);
    public static final Rotation3d TOWER_ROTATION = new Rotation3d(Units.degreesToRadians(90), 0,
        Units.degreesToRadians(90));
    public static final double ARM_INITIAL_ROLL = Units.degreesToRadians(-10);
    public static final double ARM_YAW = Units.degreesToRadians(90);
    public static final double ARM_PITCH = Units.degreesToRadians(0);
    public static final double PIVOT_MIN_SIM_ANGLE = Units.degreesToRadians(0);
    public static final double PIVOT_MAX_SIM_ANGLE = Units.degreesToRadians(180);
    public static final double PIVOT_RESTING_ANGLE = Units.degreesToRadians(20);

    public static final double PIVOT_MAX_ANGLE_DEGREES = 134;
    public static final double PIVOT_OFFSET_DEGREES = 86;

    public static final int LEFT_MOTOR_ID = 25;
    public static final int RIGHT_MOTOR_ID = 26;
    public static final int ENCODER_CHANNEL = 3;

    public static final double MAX_PIVOT_ERROR = 5;
    public static final double AMP_ANGLE_SETPOINT = 113;
    public static final double SOURCE_PICKUP_ANGLE_SETPOINT = 106;
    public static final double SUBWOOFER_ANGLE_SETPOINT = 38;
    public static final double WING_ANGLE_SETPOINT = 9.5;
    public static final double PODIUM_ANGLE_SETPOINT = 21;
    public static final double PASS_ANGLE_SETPOINT = 26;
    public static final double BACKSHOT_PODIUM_ANGLE_SETPOINT = Units.radiansToDegrees(1.77);
    public static final double BACKSHOT_SUBWOOFER_ANGLE_SETPOINT = Units.radiansToDegrees(2.22);
    public static final double GROUND_PICKUP_ANGLE = 0;
    public static final double INPUT_DEADBAND = 0.1;
    public static final InterpolatingDoubleTreeMap shotAngleMap = new InterpolatingDoubleTreeMap();
    static {
      shotAngleMap.put(1.04, 40.0);
      shotAngleMap.put(1.25, 38.0);
      shotAngleMap.put(1.5, 32.0);
      shotAngleMap.put(1.75, 28.0);
      shotAngleMap.put(2.0, 26.0);
      shotAngleMap.put(2.25, 23.5);
      shotAngleMap.put(2.5, 21.5);
      shotAngleMap.put(2.75, 19.25);
      shotAngleMap.put(2.94, 18.15);
      shotAngleMap.put(3.15, 16.65);
      shotAngleMap.put(3.55, 14.75);
      shotAngleMap.put(3.75, 14.1);
      shotAngleMap.put(4.0, 13.75);
      shotAngleMap.put(4.25, 12.8);
      shotAngleMap.put(4.5, 11.6);
    }
  }

  public static class VisionConstants {
    public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public static final Transform3d ROBOT_TO_FRONT_CAMERA = new Transform3d(Units.inchesToMeters(13.916),
        Units.inchesToMeters(3.102475), Units.inchesToMeters(7.820), new Rotation3d(0, Units.degreesToRadians(-35), 0));
    public static final Transform3d ROBOT_TO_REAR_CAMERA = new Transform3d(Units.inchesToMeters(-13.193037),
        Units.inchesToMeters(-9.543), Units.inchesToMeters(7.820),
        new Rotation3d(0, Units.degreesToRadians(-35), Units.degreesToRadians(180)));
    public static final double VISION_ODOMETRY_DIFFERENCE_FILTER_THRESHOLD = 5;
    public static final int CAMERA_RES_WIDTH = 1280;
    public static final int CAMERA_RES_HEIGHT = 960;
    public static final int CAMERA_FOV_DEG = 70;
    public static final double CAMERA_AVG_LATENCY_MS = 35;
    public static final double AVG_ERROR_PX = 0.25;
    public static final double ERROR_STDEV_PX = 0.08;
    public static final double FPS = 20;
    public static final double CAMERA_LATENCY_STDEV_MS = 5;

    public static final double APRIL_TAG_NUMBER_CONFIDENCE_SCALE = 3; // Higher makes confidence lower at each number of
                                                                      // AprilTags
    public static final double APRIL_TAG_NUMBER_EXPONENT = -1
        / (APRIL_TAG_NUMBER_CONFIDENCE_SCALE * Math.log(APRIL_TAG_NUMBER_CONFIDENCE_SCALE));
    public static final double APRIL_TAG_AREA_CONFIDENCE_SCALE = 1.7; // Higher makes confidence lower at each area of
                                                                      // AprilTags
    // See https://www.desmos.com/calculator/i5z7ddbjy4

    public static final double AMBIGUITY_TO_STDEV_EXP = 1;
    public static final Vector<N3> BASE_STDEV = VecBuilder.fill(0.1, 0.1, 1000.0); // x, y, angle
    public static final double AMBIGUITY_ACCEPTANCE_THRESHOLD = 0.2; 
    public static final double REPROJECTION_ERROR_REJECTION_THRESHOLD = 0.4;

    public static final Transform3d ROBOT_TO_NOTE_CAMERA = new Transform3d(0, 0, Units.inchesToMeters(6.5), new Rotation3d());
    public static final int NOTE_CAMERA_RES_WIDTH = 1920;
    public static final int NOTE_CAMERA_RES_HEIGHT = 1080;
    public static final double NOTE_CAMERA_FOV_DEG = 69.34;
    public static final double NOTE_CAMERA_AVG_LATENCY_MS = 35;
    public static final double NOTE_AVG_ERROR_PX = 0.25;
    public static final double NOTE_ERROR_STDEV_PX = 0.08;
    public static final double NOTE_FPS = 20;
    public static final double NOTE_CAMERA_LATENCY_STDEV_MS = 5;
    public static final double NOTE_HORIZONTAL_FOV_DEG = 62.14;
  }

  public static class IndexerConstants {
    public static final int INDEXER_CURRENT_LIMIT = 40;
    public static final int INDEXER_MOTOR_ID = 44;
    public static final double FAST_INDEXER_MOTOR_SPEED = -1;
    public static final double SLOW_INDEXER_MOTOR_OUTTAKE_SPEED = 0.25;
    public static final double DEBOUNCE_TIME = 0.05;
  }

  public static class TempConstants {
    public static final int OVERHEAT_TEMP = 80;
    public static final int SAFE_TEMP = 80;
  }

  public static class FieldConstants {
    public static final double FIELD_LENGTH = 16.54;
    public static final Pose3d BLUE_ALLIANCE_SPEAKER_POSE3D = new Pose3d(0.225, 5.55, 2.1,
        new Rotation3d(0, 0, Units.degreesToRadians(180)));
    public static final Pose2d BLUE_ALLIANCE_AMP_POSE2D = new Pose2d(2, 8.25, Rotation2d.fromDegrees(90));
    public static final Pose2d BLUE_ALLIANCE_SOURCE_POSE2D = new Pose2d(14.75, 0.75, Rotation2d.fromDegrees(-60));
    public static final Pose2d[] MIDLINE_NOTES_STARTING_POSES = new Pose2d[] {
        new Pose2d(8.258, 7.462, new Rotation2d()), new Pose2d(8.258, 5.785, new Rotation2d()),
        new Pose2d(8.258, 4.109, new Rotation2d()), new Pose2d(8.258, 2.432, new Rotation2d()),
        new Pose2d(8.258, 0.756, new Rotation2d()) };
    public static final Pose2d[] BLUE_WING_NOTES_STARTING_POSES = new Pose2d[] {
        new Pose2d(2.884, 4.109, new Rotation2d()), new Pose2d(2.884, 5.557, new Rotation2d()),
        new Pose2d(2.884, 7.004, new Rotation2d()) };
    public static final Pose3d[] NOTES_SIM_POSES = new Pose3d[] { new Pose3d(2.884, 4.109, 0, new Rotation3d()),
        new Pose3d(2.884, 5.557, 0, new Rotation3d()), new Pose3d(2.884, 7.004, 0, new Rotation3d()),
        new Pose3d(8.258, 7.462, 0, new Rotation3d()), new Pose3d(8.258, 5.785, 0, new Rotation3d()),
        new Pose3d(8.258, 4.109, 0, new Rotation3d()), new Pose3d(8.258, 2.432, 0, new Rotation3d()),
        new Pose3d(8.258, 0.756, 0, new Rotation3d()) };
    public static final double PICKUP_OFFSET = 1;
    public static final double WING_LINE_X_METERS = 5.8217054 - 1;
    public static final Pose2d[] SHOOTING_POSES = new Pose2d[]{new Pose2d(WING_LINE_X_METERS, 7.004, new Rotation2d()), new Pose2d(WING_LINE_X_METERS, 0.756, new Rotation2d())};
    public static final Pose2d SUBWOOFER_SHOOTING_POSE = new Pose2d(0.96, 6, new Rotation2d());

    public static final double NOTE_RADIUS = Units.inchesToMeters(6);
    public static final double NOTE_THICKNESS_RADIUS = Units.inchesToMeters(1);

    public static final double NOTE_AUTO_PICKUP_OVERSHOOT = Units.feetToMeters(1);
  }

  public static class AlertConstants {
    public static final double LOW_BATTERY_VOLTAGE = 11.5;
    public static final int ENDGAME_ALERT_1_TIME = 45;
    public static final int ENDGAME_ALERT_2_TIME = 30;
  }

  public static class LEDConstants {
    public static final int LED_PORT = 0;
    
    public static class LengthConstants {
        // LOWER_LEFT MID_SEGMENT LOWER_RIGHT UPPER_RIGHT UPPER_LEFT
        public static final int LOWER_LEFT = 23;
        public static final int MID = 21;
        public static final int LOWER_RIGHT = 24;
        public static final int UPPER_RIGHT = 18;
        public static final int UPPER_LEFT = 16;

        public static final int TOTAL = LOWER_LEFT + MID + LOWER_RIGHT
                + UPPER_RIGHT + UPPER_LEFT;
    }

    public static class ColorConstants {
        public static final Color LOADING = Color.kWhite;
        public static final Color SUCCESS = new Color(77, 255, 79);
        public static final Color RED = new Color(255, 25, 25);
        public static final Color PINK = new Color(255, 69, 70);
        public static final Color BLUE = new Color(25, 25, 255);
        public static final Color TEAL = new Color(160, 170, 255);
        public static final Color AUTON_1 = new Color(255, 69, 118);
        public static final Color AUTON_2 = new Color(255, 30, 180);
        public static final Color AUTON_3 = new Color(100, 25, 25);
        public static final Color USER_SIGNAL = Color.kWhite;

        public static final Color PRIDE_RED = Color.kRed;
        public static final Color PRIDE_ORANGE = Color.kOrangeRed;
        public static final Color PRIDE_YELLOW = Color.kYellow;
        public static final Color PRIDE_GREEN = Color.kGreen;
        public static final Color PRIDE_BLUE = Color.kBlue;
        public static final Color PRIDE_PURPLE = Color.kPurple;
        public static final Color TRANS_PINK = Color.kDeepPink;
        public static final Color TRANS_TEAL = new Color(0.15, 0.3, 1.0);
    }
  }
}
