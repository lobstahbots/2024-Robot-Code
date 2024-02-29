// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SimConstants;

public class SwerveModuleSim implements SwerveModuleIO {
  /** Creates a new SwerveModuleSim. */
  private FlywheelSim simDriveMotor = new FlywheelSim(DCMotor.getNEO(1), RobotConstants.DRIVE_GEAR_RATIO, 0.025);
  private FlywheelSim simAngleMotor = new FlywheelSim(DCMotor.getNEO(1), RobotConstants.ANGLE_GEAR_RATIO, 0.025);

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private Rotation2d angularOffset;

  public SwerveModuleSim(double angularOffsetDegrees) {
    this.angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    simDriveMotor.update(SimConstants.LOOP_TIME);
    simAngleMotor.update(SimConstants.LOOP_TIME);

    double angleDelta = simAngleMotor.getAngularVelocityRadPerSec() * 0.020;

    inputs.turnPosition = Rotation2d.fromRadians(inputs.turnPosition.getRadians() + angleDelta + angularOffset.getRadians());
    inputs.turnPosition = Rotation2d.fromRadians(inputs.turnPosition.getRadians());
    inputs.drivePosition = Rotation2d.fromRadians(inputs.drivePosition.getRadians() + simDriveMotor.getAngularVelocityRadPerSec() * SimConstants.LOOP_TIME);
    inputs.driveVelocityRadPerSec = simDriveMotor.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] { Math.abs(simDriveMotor.getCurrentDrawAmps()) };
    inputs.turnVelocityRadPerSec = simAngleMotor.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] { Math.abs(simAngleMotor.getCurrentDrawAmps()) };
  }

  /**Sets voltage of driving motor.
   * 
   * @param volts The voltage the motor should be set to.
   */
  public void setDriveVoltage(double volts) {
    simDriveMotor.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  /**Sets voltage of turn motor.
   * 
   * @param volts The voltage the motor should be set to.
   */
  public void setTurnVoltage(double volts) {
    simAngleMotor.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

}
