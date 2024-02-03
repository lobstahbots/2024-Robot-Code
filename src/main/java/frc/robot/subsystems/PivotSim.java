// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
          Units.degreesToRadians(90 - 180),
          Units.degreesToRadians(135 - 180),
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
    SmartDashboard.putData("Pivot", pivot);
  }

   public void updateInputs(PivotIOInputs inputs) {
    inputs.motorLeftCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.motorRightCurrentAmps = pivotSim.getCurrentDrawAmps();
    inputs.position = Rotation2d.fromRadians(pivotSim.getAngleRads());
    inputs.velocity = Rotation2d.fromRadians(pivotSim.getVelocityRadPerSec());
    // System.out.println(inputs.position);
    // arm.setAngle(inputs.position);
  }

  public void setVoltage(double voltage) {
    pivotSim.setInput(VecBuilder.fill(voltage));
    
    arm.setAngle(270 - Rotation2d.fromRadians(pivotSim.getAngleRads()).getDegrees());
    pivotSim.update(SimConstants.LOOP_TIME);
    // SmartDashboard.putData("Pivot", pivot);
    Logger.recordOutput("Arm", pivot);
    Logger.recordOutput("ArmPose", new Pose3d(-Units.inchesToMeters(18),Units.inchesToMeters(-2), 0, new Rotation3d(Math.PI - pivotSim.getAngleRads(), Units.degreesToRadians(180), Units.degreesToRadians(90))));
    Logger.recordOutput("IntakePose", new Pose3d(0,0, Units.inchesToMeters(14), new Rotation3d(Units.degreesToRadians(240) - pivotSim.getAngleRads(), Units.degreesToRadians(180), Units.degreesToRadians(90))));
  }

  public void stopPivot() {
    pivotSim.setInput(0);
  }
}