// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frc7153.commands.TeleopCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.led.DriverStationLEDCommand;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  // Subsystems
  private SwerveBase driveBase = new SwerveBase();
  //private Shooter shooter = new Shooter();
  //private Intake intake = new Intake();
  private LED led = new LED();

  // Controls
  private XboxController driverXboxController = new XboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Teleop Drive Command
    driveBase.setDefaultCommand(new TeleopCommand(
      new TeleopDriveCommand(driveBase, 
        driverXboxController::getLeftY, 
        driverXboxController::getLeftX,
        driverXboxController::getRightX
      )
    ));
    
    
    
    //led.setDefaultCommand(new DriverStationLEDCommand(led));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
