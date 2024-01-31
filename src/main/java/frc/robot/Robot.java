// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frc7153.logging.LoggingUtil;

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
  //private Intake intake = new Intake();

  private GenericEntry driveStateOut = Shuffleboard.getTab("Swerve").add("Drive State", "?").getEntry();
  private GenericEntry drivePosOut = Shuffleboard.getTab("Swerve").add("Drive Pos", "?").getEntry();
  private GenericEntry driveSPOut = Shuffleboard.getTab("Swerve").add("Steer SP", "?").getEntry();

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

    // Output swerve test
    driveStateOut.setString(module.getState().toString());
    drivePosOut.setString(module.getPosition().toString());
    driveSPOut.setString(module.getSetpoint().toString());
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
    // Test swerve
    module.setDriveWheelVelocity(joystick.getY() * 5.0);
    module.setSteerAngle(joystick.getThrottle() * 185.0);

    /*if (joystick.getTrigger()) {
      intake.enableIntake();
    } else {
      intake.end();
    }*/
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
