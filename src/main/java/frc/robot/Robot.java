// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frc7153.logging.LoggingUtil;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.util.CANLogger;
import frc.robot.util.ConsoleLogger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  // TESTING
  private SwerveModule module = new SwerveModule( // FL
            HardwareConstants.kFL_DRIVE_CAN, HardwareConstants.kFL_STEER_CAN, 
            HardwareConstants.kFL_CANCODER, DriveConstants.kFL_STEER_ZERO
        );
  private Joystick joystick = new Joystick(0);

  private GenericEntry driveVeloOut = Shuffleboard.getTab("Swerve").add("Drive Velocity", 0.0).getEntry();
  private GenericEntry drivePosOut = Shuffleboard.getTab("Swerve").add("Drive Pos", 0.0).getEntry();
  private GenericEntry steerPoseOut = Shuffleboard.getTab("Swerve").add("Steer Pos", 0.0).getEntry();

  @Override
  public void robotInit() {
    // Start logging
    DataLogManager.start();
    ConsoleLogger.init();
    DriverStation.startDataLog(DataLogManager.getLog(), true);

    // Log metadata
    LoggingUtil.addDefaultMetadata();

    // Create container
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Periodic logging methods
    CANLogger.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
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
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    // Output swerve test
    steerPoseOut.setDouble(module.getState().angle.getDegrees());
    drivePosOut.setDouble(module.getPosition().distanceMeters);
    driveVeloOut.setDouble(module.getState().speedMetersPerSecond);

    // Test swerve
    module.setDriveWheelVelocity(joystick.getThrottle() * 5.0);
    System.out.println(joystick.getThrottle() * 5.0);
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
