// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;
import org.littletonrobotics.junction.wpilog.WPILOGReader;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.SimConstants;
import frc.robot.networkalerts.Alert;
import frc.robot.networkalerts.Alert.AlertType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    private Alert canAlert = new Alert("CAN Error", AlertType.ERROR);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        Pathfinding.setPathfinder(new LocalADStarAK());
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("Lobstah Bots", "2024 Robot Code");

        File log = new File(Filesystem.getOperatingDirectory(), "log");
        String logPath = log.getAbsolutePath();

        if (Robot.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Save outputs to a new log
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        } else {
            if (SimConstants.REPLAY) {
                String replayPath = logPath + "\\" + SimConstants.REPLAY_LOG_PATH;
                Logger.setReplaySource(new WPILOGReader(replayPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(replayPath, "_replay")));
                setUseTiming(false); // Run as fast as possible
            } else {
                Logger.addDataReceiver(new WPILOGWriter(logPath, 0.02)); // Save outputs to a new log
            }
        }
        DataLogManager.start();
        URCL.start();
        Logger.start();

        // PortForwarder.add(5801, "photonvision1.local", 5800);
        // PortForwarder.add(5802, "photonvision2.local", 5800);
        // PortForwarder.add(1183, "10.2.46.11", 1183);
        // PortForwarder.add(1184, "10.2.46.11", 1184);
        // PortForwarder.add(1189, "10.2.46.12", 1189);
        // PortForwarder.add(1190, "10.2.46.12", 1190);

        // Logger.getInstance().disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page

        m_robotContainer = new RobotContainer();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran during disabled, autonomous, teleoperated
     * and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        CANStatus canStatus = RobotController.getCANStatus();
        if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
            canAlert.set(true);
            canAlert.setText(String.format("CAN error: %d receive errors, %d transmit errors, %d%% utilization",
                    canStatus.receiveErrorCount, canStatus.transmitErrorCount, canStatus.percentBusUtilization));
        } else canAlert.set(false);
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        m_robotContainer.setIdleMode(IdleMode.kBrake, NeutralModeValue.Brake);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        m_robotContainer.setIdleMode(IdleMode.kBrake, NeutralModeValue.Coast);
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        m_robotContainer.setIdleMode(IdleMode.kCoast, NeutralModeValue.Coast);
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
