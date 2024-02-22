// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.SimConstants;

/** Add your docs here. */
public class PivotSim implements PivotIO {
  private final DCMotor armGearbox = DCMotor.getNEO(2);

  private final Mechanism2d pivot = new Mechanism2d(1, 1);
  private final MechanismRoot2d root = pivot.getRoot("pivot", Units.inchesToMeters(14), Units.inchesToMeters(17.5));
  private final MechanismLigament2d arm;
  private final MechanismLigament2d shooter;
  private final MechanismLigament2d shooterIndexer;

  private SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          armGearbox,
          80,
          SingleJointedArmSim.estimateMOI(Units.inchesToMeters(17.18), Units.lbsToKilograms(16)),
          16,
          PivotConstants.PIVOT_MIN_ANGLE,
          PivotConstants.PIVOT_MAX_ANGLE,
          true,
          0,
          VecBuilder.fill(2.0 * Math.PI / 2048) // Add noise with a std-dev of 1 tick
          );

  public PivotSim(){
    this.arm = root.append(new MechanismLigament2d("arm", Units.inchesToMeters(17.18), -45));
    this.shooter = arm.append(new MechanismLigament2d("shooter", Units.inchesToMeters(7.5), 45));
    this.shooterIndexer = arm.append(new MechanismLigament2d("shooterIndexer", Units.inchesToMeters(-7.5), 45));
    this.arm.setColor(new Color8Bit(Color.kBlue));
    this.shooter.setColor(new Color8Bit(Color.kYellow));
    this.shooterIndexer.setColor(new Color8Bit(Color.kYellow));
  }

   public void updateInputs(PivotIOInputs inputs) {
    inputs.motorLeftCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.motorRightCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.position = Rotation2d.fromRadians(pivotSim.getAngleRads());
    inputs.velocity = Rotation2d.fromRadians(pivotSim.getVelocityRadPerSec());
  }

  /** Sets voltage of the pivot sim motor.
   * @param voltage The voltage to set it to
   */
  public void setVoltage(double voltage) {
    pivotSim.setInput(VecBuilder.fill(voltage));
    arm.setAngle(270 - Rotation2d.fromRadians(pivotSim.getAngleRads()).getDegrees());
    pivotSim.update(SimConstants.LOOP_TIME);
  }

  /** Stops the pivot sim motor. */
  public void stopPivot() {
    pivotSim.setInput(0);
  }

  public void periodic() {
    var logTransform = PivotKinematics.angleToSimPivotTransform(pivotSim.getAngleRads());
    Logger.recordOutput("Arm", pivot);
    Logger.recordOutput("TowerPose", new Pose3d(PivotConstants.ORIGIN_TO_TOWER_MOUNT_X_DIST, PivotConstants.ORIGIN_TO_TOWER_MOUNT_Y_DIST, PivotConstants.ORIGIN_TO_TOWER_MOUNT_Z_DIST, PivotConstants.TOWER_ROTATION));
    Logger.recordOutput("ArmPose", new Pose3d(PivotConstants.ORIGIN_TO_ARM_MOUNT_X_DIST, PivotConstants.ORIGIN_TO_ARM_MOUNT_Y_DIST, PivotConstants.ORIGIN_TO_ARM_MOUNT_Z_DIST + logTransform.getY(), new Rotation3d(PivotConstants.ARM_INITIAL_ROLL - pivotSim.getAngleRads(), PivotConstants.ARM_PITCH, PivotConstants.ARM_YAW)));
  }
}