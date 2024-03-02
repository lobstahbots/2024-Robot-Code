// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter.
   * @param io The {@link ShooterIO} used to construct the Intake.
   */
    private ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;

  }
  /**
   * Sets the intake motor speed to the given speed
   * @param shooterMotorSpeed
   */
  public void setShooterSpeed(double upperShooterSpeed, double lowerShooterSpeed) {
    io.setShooterSpeed(upperShooterSpeed, lowerShooterSpeed);
  }
  
  /** Stops the intake motor. */
  public void stopMotor() {
    io.stopMotor();
  }
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    io.periodic();
  }
}
