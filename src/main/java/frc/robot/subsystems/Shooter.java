// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  private final CANSparkMax upperShooterMotor;
  private final CANSparkMax lowerShooterMotor;

  /**
   * Creates a new Shooter subsystem.
   * @param upperShooterMotorID The CAN ID of the upper shooter motor
   * @param lowerShooterMotorID The CAN ID of the lower shooter motor
   */
  public Shooter(int upperShooterMotorID, int lowerShooterMotorID) {
    this.upperShooterMotor = new CANSparkMax(upperShooterMotorID, MotorType.kBrushless);
    this.lowerShooterMotor = new CANSparkMax(lowerShooterMotorID, MotorType.kBrushless);
    this.upperShooterMotor.setInverted(true);
    this.lowerShooterMotor.setInverted(false);
    this.upperShooterMotor.setIdleMode(IdleMode.kBrake);
    this.lowerShooterMotor.setIdleMode(IdleMode.kBrake);
    this.upperShooterMotor.setSmartCurrentLimit(40);
    this.lowerShooterMotor.setSmartCurrentLimit(40);
  }

  /**
   * Spins the shooter motors at the given speed.
   * @param shooterSpeed The speed to set the shooter to.
   */
  public void setShooterSpeed(double upperShooterSpeed, double lowerShooterSpeed){
    upperShooterMotor.set(upperShooterSpeed);
    lowerShooterMotor.set(lowerShooterSpeed);
  }

  /**
   * Stops the shooter motors.
   */
  public void stopMotor(){
    upperShooterMotor.stopMotor();
    lowerShooterMotor.stopMotor();
  }
  
}
