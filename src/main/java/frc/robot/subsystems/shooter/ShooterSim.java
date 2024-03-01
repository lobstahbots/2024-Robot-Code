// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterSim implements ShooterIO {
  private final FlywheelSim upperShooterMotor;
  private final FlywheelSim lowerShooterMotor;

  /**
   * Creates a new simulated Shooter subsystem. 
   **/
  public ShooterSim() {
    this.upperShooterMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.001);
    this.lowerShooterMotor = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.001);
  }

  /**
   * Spins the shooter motors at the given speeds.
   * @param upperShooterSpeed the speed to set the upper shooter to
   * @param lowerShooterSpeed the speed to set the lower shooter to
   */
  public void setShooterSpeed(double upperShooterSpeed, double lowerShooterSpeed) {
    upperShooterMotor.setState(upperShooterSpeed);
    lowerShooterMotor.setState(lowerShooterSpeed);
  }

  /**
   * Stops the shooter motors.
   */
  public void stopMotor() {
    upperShooterMotor.setState(0);
    lowerShooterMotor.setState(0);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.upperShooterMotorCurrent = upperShooterMotor.getCurrentDrawAmps();
    inputs.upperShooterMotorVelocity = upperShooterMotor.getAngularVelocityRPM();
    inputs.lowerShooterMotorCurrent = lowerShooterMotor.getCurrentDrawAmps();
    inputs.lowerShooterMotorVelocity = lowerShooterMotor.getAngularVelocityRPM();
  }
}
