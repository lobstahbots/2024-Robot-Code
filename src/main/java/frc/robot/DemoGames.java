// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.DemoConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.RotatePivotCommand;
import frc.robot.commands.TurnToPointCommand;
import frc.robot.subsystems.drive.DriveBase;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

/** Add your docs here. */
public class DemoGames {
    private final Vision vision;
    private final DriveBase driveBase;
    private final Shooter shooter;
    private final Pivot pivot;
    private final Indexer indexer;

    private double acquiredNoteTime = 0;
    private double shotNoteTime = 0;
    private boolean hasNote = false;

    private Pose3d tagLocation = new Pose3d();

    public DemoGames(Vision vision, DriveBase driveBase, Shooter shooter, Pivot pivot, Indexer indexer) {
        this.vision = vision;
        this.driveBase = driveBase;
        this.shooter = shooter;
        this.pivot = pivot;
        this.indexer = indexer;
    }

    private Command getBaseCommand(DoubleSupplier pivotAngleSupplier, int fiducialId) {
        return new StartEndCommand(() -> driveBase.useVision(false), () -> driveBase.useVision(true))
                .alongWith(
                        new TurnToPointCommand(driveBase, driveBase::getPose, () -> tagLocation.toPose2d(), () -> 0,
                                () -> 0, () -> true, false),
                        new RotatePivotCommand(pivot, pivotAngleSupplier), new RunCommand(() -> {
                            var targetOptional = vision.getFrontTargets().stream()
                                    .filter(target -> target.getFiducialId() == fiducialId).findFirst();
                            targetOptional.ifPresent(target -> {
                                tagLocation = new Pose3d(driveBase.getPose())
                                        .plus(VisionConstants.ROBOT_TO_FRONT_CAMERA)
                                        .plus(target.getBestCameraToTarget());
                            });
                            targetOptional = vision.getRearTargets().stream()
                                    .filter(target -> target.getFiducialId() == fiducialId).findFirst();
                            targetOptional.ifPresent(target -> {
                                tagLocation = new Pose3d(driveBase.getPose()).plus(VisionConstants.ROBOT_TO_REAR_CAMERA)
                                        .plus(target.getBestCameraToTarget());
                            });
                        }));
    }

    public Command getFrisbeeCommand() {
        return getBaseCommand(
                () -> Units.radiansToDegrees(
                        Math.atan2(tagLocation.minus(new Pose3d(driveBase.getPose())).getTranslation().getNorm(),
                                tagLocation.getZ() - PivotConstants.ORIGIN_TO_ARM_MOUNT_Z_DIST)),
                DemoConstants.FRISBEE_FIDUCIAL_ID).alongWith(new RunCommand(() -> {
                    if (indexer.intakeBeamBroken() || indexer.flywheelBeamBroken()) {
                        if (!hasNote) {
                            hasNote = true;
                            acquiredNoteTime = Timer.getFPGATimestamp();
                        }
                        if (Timer.getFPGATimestamp() - acquiredNoteTime >= DemoConstants.FRISBEE_SHOOT_DELAY) {
                            shooter.setShooterSpeed(DemoConstants.FRISBEE_SHOOT_SPEED,
                                    DemoConstants.FRISBEE_SHOOT_SPEED);
                            if (shooter.getLowerFlywheelVelocityRPS() >= DemoConstants.FRISBEE_SHOOT_SPEED
                                    * ShooterConstants.SHOOTING_FLYWHEEL_VELOCITY_DEADBAND_FACTOR
                                    && shooter.getUpperFlywheelVelocityRPS() >= DemoConstants.FRISBEE_SHOOT_SPEED
                                            * ShooterConstants.SHOOTING_FLYWHEEL_VELOCITY_DEADBAND_FACTOR) {
                                indexer.setIndexerMotorSpeed(IndexerConstants.FAST_INDEXER_MOTOR_SPEED);
                            }
                        }
                    } else {
                        if (hasNote) {
                            hasNote = false;
                            shotNoteTime = Timer.getFPGATimestamp();
                        }
                        if (Timer.getFPGATimestamp() - shotNoteTime >= DemoConstants.FRISBEE_INTAKE_DELAY) {
                            shooter.setShooterSpeed(ShooterConstants.UNSHOOTER_SPEED, ShooterConstants.UNSHOOTER_SPEED);
                        }
                    }
                }, shooter, indexer));
    }

    public Command getPoleCommand() {
        return getBaseCommand(() -> 0, DemoConstants.POLE_FIDUCIAL_ID);
    }
}
