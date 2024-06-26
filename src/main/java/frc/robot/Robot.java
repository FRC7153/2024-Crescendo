// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frc7153.logging.LoggingUtil;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.CANLogger;
import frc.robot.util.ConsoleLogger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Start logging
    DataLogManager.start();
    ConsoleLogger.robotProgramRunning();
    DriverStation.startDataLog(DataLogManager.getLog(), true);

    // Don't warn joystick unplugged (spams console)
    DriverStation.silenceJoystickConnectionWarning(true);

    // Log metadata
    LoggingUtil.addDefaultMetadata();

    // Create container
    m_robotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    // Do not command in test!
    if (!DriverStation.isTestEnabled()) CommandScheduler.getInstance().run();

    // Periodic logging methods
    CANLogger.periodic();
    m_robotContainer.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Check swerves
    m_robotContainer.recheckSwerveHeadings();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    // Restart motion profile
    m_robotContainer.resetArmMotionProfile();

    // Check swerves
    m_robotContainer.recheckSwerveHeadings();

    // Recheck limelight
    m_robotContainer.setLimelightPriorityTag();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.testInit();
  }

  @Override
  public void testPeriodic() {
    m_robotContainer.testExec();
  }

  @Override
  public void testExit() {
    m_robotContainer.testEnd();
  }
}
