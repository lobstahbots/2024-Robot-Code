// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.swervemodules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SimConstants;

public class SwerveModuleIOSim implements SwerveModuleIO {
  /** Creates a new SwerveModuleSim. */
  private DCMotorSim simDriveMotor = new DCMotorSim(DCMotor.getNEO(1), RobotConstants.DRIVE_GEAR_RATIO, 0.025);
  private DCMotorSim simAngleMotor = new DCMotorSim(DCMotor.getNEO(1), RobotConstants.ANGLE_GEAR_RATIO, 0.025);

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  private Rotation2d angularOffset;

  public SwerveModuleIOSim(double angularOffsetDegrees) {
    this.angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees);
  }

  public void updateInputs(ModuleIOInputs inputs) {
    simDriveMotor.update(SimConstants.LOOP_TIME);
    simAngleMotor.update(SimConstants.LOOP_TIME);

    if (DriverStation.isDisabled()) {
      setDriveVoltage(0);
      setTurnVoltage(0);
    }

    inputs.turnPosition = Rotation2d.fromRadians(simAngleMotor.getAngularPositionRad() + angularOffset.getRadians());
    inputs.drivePosition = Rotation2d.fromRadians(simDriveMotor.getAngularPositionRad() + simDriveMotor.getAngularVelocityRadPerSec() * SimConstants.LOOP_TIME);
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
