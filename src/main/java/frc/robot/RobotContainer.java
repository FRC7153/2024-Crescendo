// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.frc7153.commands.TeleopCommand;
//import com.frc7153.commands.UnrequiredConditionalCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.Autonomous;
import frc.robot.commands.ArmAmpCommand;
import frc.robot.commands.ArmSpeakerCommand;
import frc.robot.commands.ClimberStageCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LoadShooterCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.led.DriverStationLEDCommand;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  // Subsystems
  private SwerveBase driveBase = new SwerveBase();
  private Shooter shooter = new Shooter();
  private Indexer indexer = new Indexer();
  private Intake intake = new Intake();
  private Arm arm = new Arm();
  private Climber climber = new Climber();
  //private LED led = new LED();

  // Auto
  private Autonomous auto = new Autonomous(shooter);

  // Controls
  private CommandXboxController driverXboxController = new CommandXboxController(0);
  private CommandJoystick operatorController = new CommandJoystick(1);

  public RobotContainer() {
    // Default
    driveBase.initDefaultCommand(driverXboxController);
    arm.initDefaultCommand();
    shooter.initDefaultCommand();
    indexer.initDefaultCommand();
    
    configureBindings();
  }

  private void configureBindings() {
    // Operator Arm Button (2) pressed while robot is LOADED
    operatorController.button(2).and(StateController.getLoadedTrigger())
      .whileTrue(new ConditionalCommand(
        new ArmSpeakerCommand(shooter), // Throttle up, speaker
        new ArmAmpCommand(shooter), // Throttle down, amp
        () -> operatorController.getThrottle() < 0.0
      ));

    // Operator Arm Button (2) pressed while robot is NOT LOADED
    /*operatorController.button(2).and(StateController.getLoadedTrigger().negate())
      .whileTrue(new InstantCommand()); // TODO SOURCE PICKUP

    // Operator Arm Button (2) released while robot is NOT LOADED
    operatorController.button(2).negate().and(StateController.getLoadedTrigger().negate())
      .whileTrue(new InstantCommand()); // TODO GROUND PICKUP*/

    // Operator Shoot Button Trigger while robot is LOADED
    operatorController.trigger().and(StateController.getLoadedTrigger())
      .onTrue(new ShootCommand(indexer, true, false));
    
    // Operator Climb Button (3) pressed
    operatorController.button(3).onTrue(new ClimberStageCommand(climber, true));

    // Operator CLimb Button (4) pressed
    operatorController.button(4).onTrue(new ClimberStageCommand(climber, false));
    
    // Handle piece intaking
    new Trigger(() -> (!StateController.getState().equals(NoteState.LOADED) && DriverStation.isTeleopEnabled()))
      // Robot is EMPTY or PROCESSING
      .whileTrue(new IntakeCommand(intake, true))
      .whileTrue(new LoadShooterCommand(shooter, indexer))
      // Robot is LOADED
      .whileFalse(new IntakeCommand(intake, false));
    
    //led.setDefaultCommand(new DriverStationLEDCommand(led));
  }

  public Command getAutonomousCommand() {
    return auto.getSelected();
  }
}
