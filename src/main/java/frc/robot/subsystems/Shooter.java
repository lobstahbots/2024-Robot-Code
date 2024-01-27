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
  private final CANSparkMAX lowerShooterMotor;
  /** Creates a new Shooter. */
  public Shooter(int upperShooterMotorID, int lowerShooterMotorID) {
    this.upperShooterMotor = new CANSparkMax(upperShooterMotorID, MotorType.kBrushless);
    this.lowerShooterMotor = new CANSparkMax(lowerShooterMotorID, MotorType.kBrushless);
    this.upperShooterMotor.setInverted(true);
    this.lowerShooterMotor.setInverted(false);
    this.upperShooterMotor.setSmartCurrentLimit(40);
    this.lowerShooterMotor.setSmartCurrentLimit(40);
  }
  public void setShooterSpeed(double shooterSpeed){
    upperShooterMotor.set(shooterSpeed);
    lowerShooterMotor.set(shooterSpeed);
  }
  public void stopMotor(){
    upperShooterMotor.stopMotor();
    lowerShooterMotor.stopMotor();
  }
  
}
