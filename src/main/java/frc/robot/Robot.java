// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsReal;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private final LEDs leds = new LEDs(new LEDsReal(LEDConstants.LED_PORT, LEDConstants.LED_LENGTH));
  private LEDsDemo ledsDemo = null;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  //   // Record metadata
  //   Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
  //   Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
  //   Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
  //   Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
  //   Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
  //   Logger.recordMetadata("Lobstah Bots", "2024 Robot Code");

  //   File log = new File (Filesystem.getOperatingDirectory(), "log");
  //   String logPath = log.getAbsolutePath();

  //   if (Robot.isReal()) {
  //     Logger.addDataReceiver(new WPILOGWriter(logPath)); // Save outputs to a new log
  //     Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
  // } else {
  //   if(SimConstants.REPLAY) {
  //     String replayPath = logPath + "\\Log_24-02-22_02-19-32.wpilog";
  //     Logger.setReplaySource(new WPILOGReader(replayPath));
  //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(replayPath, "_replay")));
  //   } else {
  //     setUseTiming(false); // Run as fast as possible
  //     Logger.addDataReceiver(new WPILOGWriter(logPath, 0.02)); // Save outputs to a new log
  //   }
  // }
  
  // // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
  //  Logger.start();

    try {
      Thread.sleep(5000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    if (ledsDemo != null) {
      ledsDemo.interrupt();
      try {
        ledsDemo.join(30);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      if (ledsDemo.isAlive()) return;
    }
    ledsDemo = new LEDsDemo(leds);
    ledsDemo.start();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
