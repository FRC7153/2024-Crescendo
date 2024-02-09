// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frc7153.commands.TeleopCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.Autonomous;
import frc.robot.commands.ArmShooterCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.led.DriverStationLEDCommand;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  // Subsystems
  private SwerveBase driveBase = new SwerveBase();
  private Shooter shooter = new Shooter();
  //private Intake intake = new Intake();
  //private Arm arm = new Arm();
  //private Climber climber = new Climber();
  //private LED led = new LED();

  // Auto
  private Autonomous auto = new Autonomous(shooter);

  // Controls
  private CommandXboxController driverXboxController = new CommandXboxController(0);
  private CommandJoystick operatorController = new CommandJoystick(1);

  public RobotContainer() {
    // Default
    shooter.setDefaultCommand();
    
    configureBindings();
  }

  private void configureBindings() {
    // Teleop Drive Command
    driveBase.setDefaultCommand(new TeleopCommand(
      new TeleopDriveCommand(driveBase, 
        driverXboxController::getLeftY, 
        driverXboxController::getLeftX,
        driverXboxController::getRightX
      ),
      new TeleopDriveCommand(driveBase, () -> 0.0, () -> 0.0, () -> 0.0)
    ));
    
    // Operator Arm Button (2)
    operatorController.button(2).whileTrue(new ArmShooterCommand(shooter, false));
    
    //led.setDefaultCommand(new DriverStationLEDCommand(led));
  }

  public Command getAutonomousCommand() {
    return auto.getSelected();
  }
}
