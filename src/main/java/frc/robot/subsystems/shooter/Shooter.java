// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter.
   * @param io The {@link ShooterIO} used to construct the Intake.
   */
    private ShooterIO io;
    private double desiredSpeed = 0;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
     private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    ShooterConstants.KS, ShooterConstants.KV, ShooterConstants.KA
  );
  private final ProfiledPIDController upperController = new ProfiledPIDController(
    ShooterConstants.PID_P,
    ShooterConstants.PID_I,
    ShooterConstants.PID_D,
    new TrapezoidProfile.Constraints(ShooterConstants.MAX_VELOCITY, ShooterConstants.MAX_ACCELERATION)
  );
  private final ProfiledPIDController lowerController = new ProfiledPIDController(
    ShooterConstants.PID_P,
    ShooterConstants.PID_I,
    ShooterConstants.PID_D,
    new TrapezoidProfile.Constraints(ShooterConstants.MAX_VELOCITY, ShooterConstants.MAX_ACCELERATION)
  );

  public Shooter(ShooterIO io) {
    this.io = io;
    upperController.setTolerance(ShooterConstants.PID_TOLERANCE);
    lowerController.setTolerance(ShooterConstants.PID_TOLERANCE);
  }
  /**
   * Sets the intake motor speed to the given speed
   * @param shooterMotorSpeed
   */
  public void setShooterSpeed(double upperShooterMotorSpeed, double lowerShooterMotorSpeed) {
    lowerShooterMotorSpeed *= 100;
    upperShooterMotorSpeed *= 100;
    desiredSpeed = upperShooterMotorSpeed;
    lowerController.setGoal(lowerShooterMotorSpeed);
    upperController.setGoal(upperShooterMotorSpeed);
    resetControllerError(upperShooterMotorSpeed, lowerShooterMotorSpeed);
    double pidOutputLower = lowerController.calculate(inputs.lowerShooterMotorVelocity, lowerShooterMotorSpeed);
    double pidOutputUpper = upperController.calculate(inputs.upperShooterMotorVelocity, upperShooterMotorSpeed);
    double feedforwardOutputUpper = feedforward.calculate(upperController.getSetpoint().velocity);
    double feedforwardOutputLower = feedforward.calculate(lowerController.getSetpoint().velocity);
    double outputUpper = pidOutputUpper + feedforwardOutputUpper;
    double outputLower = pidOutputLower + feedforwardOutputLower;
    outputLower = Math.abs(outputLower) * Math.signum(lowerShooterMotorSpeed);
    outputUpper = Math.abs(outputUpper) * Math.signum(upperShooterMotorSpeed);
    io.setShooterSpeed(outputUpper, outputLower);
  }

  public void resetControllerError(double upperSpeed, double lowerSpeed) {
    upperController.reset(upperSpeed);
    lowerController.reset(lowerSpeed);
  }

  /** Stops the shooter motor. */
  public void stopMotor() {
    io.stopMotor();
    desiredSpeed = 0;
  }

  public void setIdleMode(NeutralModeValue shooterIdleMode) {
    io.setIdleMode(shooterIdleMode);
  }

  public double getLowerFlywheelVelocityRPS() {
    return inputs.lowerShooterMotorVelocity;
  }

  public double getUpperFlywheelVelocityRPS() {
    return inputs.upperShooterMotorVelocity;
  }

  public double getSetpoint() {
    return desiredSpeed;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    io.periodic();
  }
}
