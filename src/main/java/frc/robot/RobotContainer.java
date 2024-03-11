// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.LEDConstants;
import frc.robot.auto.Autonomous;
import frc.robot.commands.ArmSpeakerCommand;
import frc.robot.commands.ArmToStateCommand;
import frc.robot.commands.ClimberStageCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LoadShooterCommand;
import frc.robot.commands.SetStateCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.led.FlashLEDCommand;
import frc.robot.commands.led.SetLEDCommand;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.util.Dashboard;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;
import frc.robot.util.StateController.ObjectiveState;
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
  private LED led = new LED();

  // Auto
  private Autonomous auto = new Autonomous(shooter);

  // Controls
  private CommandXboxController driverXboxController = new CommandXboxController(0);
  private CommandJoystick operatorController = new CommandJoystick(1);

  // Triggers
  private Trigger isTeleop, detectingNote;
  
  private Dashboard dashboard = new Dashboard(driveBase);

  public RobotContainer() {
    // Default
    driveBase.initDefaultCommand();
    arm.initDefaultCommand();
    shooter.initDefaultCommand();
    indexer.initDefaultCommand();
    led.initDefaultCommand();
    
    // Config bindings
    configureBindings();

    // Config Auto
  }

  private void configureBindings() {
    // Teleop trigger (reused multiple times)
    isTeleop = new Trigger(DriverStation::isTeleopEnabled);

    // Drive Control
    isTeleop.whileTrue(new TeleopDriveCommand(
        driveBase, 
        () -> -driverXboxController.getLeftY(), 
        driverXboxController::getLeftX, 
        () -> -driverXboxController.getRightX()
      ).repeatedly());

    // Driver Intake Button (RT) pressed while robot is EMPTY and SCORING
    driverXboxController.rightTrigger().and(StateController.buildTrigger(NoteState.EMPTY, ObjectiveState.SCORING))
      .whileTrue(new LoadShooterCommand(shooter, indexer, led))
      .whileTrue(new IntakeCommand(intake, true));

    // Driver Source Intake Button (LT) pressed while robot is EMPTY and SCORING
    driverXboxController.leftTrigger().and(StateController.buildTrigger(NoteState.EMPTY, ObjectiveState.SCORING))
      .whileTrue(new PrintCommand("Source pickup not yet implemented")); // TODO SOURCE PICKUP
    
    // Arm to intake while robot is EMPTY and SCORING
    StateController.buildTrigger(NoteState.EMPTY, ObjectiveState.SCORING)
      .and(isTeleop) // Because this will run automatically, we should specify only in teleop
      .whileTrue(new ArmToStateCommand(arm, ArmPositions.kGROUND_INTAKE));

    // Operator Arm Speaker Button (6) pressed while robot is LOADED and SCORING
    operatorController.button(6).and(StateController.buildTrigger(NoteState.LOADED, ObjectiveState.SCORING))
      .whileTrue(new ArmSpeakerCommand(arm, shooter, led, () -> driveBase.getPosition(false)));
      /*.whileTrue(new TeleopDriveHeadingLockCommand(
        driveBase, 
        () -> -driverXboxController.getLeftY(), 
        () -> -driverXboxController.getRightX()
      ));*/ // TODO driver should have control of heading lock

    // Operator Arm Amp Button (4) pressed while robot is LOADED and SCORING
    operatorController.button(4).and(StateController.buildTrigger(NoteState.LOADED, ObjectiveState.SCORING))
      //.whileTrue(new ArmAmpCommand(arm, led, () -> driveBase.getPosition(false)));
      .whileTrue(new ArmToStateCommand(arm, ArmPositions.kFRONT_AMP, ArmPositions.kREAR_AMP, () -> driveBase.getPosition(false), 0.0, 180.0))
      .whileTrue(new ConditionalCommand(
        new SetLEDCommand(led, 0.0), 
        new InstantCommand(() -> led.setAllianceStationColor(), led),
        arm::atSetpoint
      ));

    // Operator Shoot Button (trigger) pressed while robot is LOADED and SCORING
    // Assumes shooter is already up-to-speed (if needed)
    operatorController.trigger().and(StateController.buildTrigger(NoteState.LOADED, ObjectiveState.SCORING))
      .whileTrue(new ConditionalCommand(
        new ShootCommand(indexer, true), // Shoot forwards
        new ShootCommand(indexer, false), // Shoot backwards (if placing in AMP from rear)
        shooter::isShooterRunning
      )); // Will change NoteState to EMPTY
    
    // Operator Climb Button (6) pressed while robot is CLIMBING
    operatorController.button(6).and(StateController.buildTrigger(null, ObjectiveState.CLIMBING))
      .onTrue(new ClimberStageCommand(climber, 67.0)); // 70.0

    // Operator Climb Button (4) pressed while robot is CLIMBING
    operatorController.button(4).and(StateController.buildTrigger(null, ObjectiveState.CLIMBING))
      .onTrue(new ClimberStageCommand(climber, 0.0));
    
    //Reverse Intake
    driverXboxController.rightBumper().whileTrue(new IntakeCommand(intake, false));
    
    // Handle Objective State Control (Operator throttle, axis 3)
    operatorController.axisLessThan(3, -2.0/3.0)
      .and(isTeleop)
      .and(operatorController.getHID()::isConnected)
      .onTrue(new InstantCommand(() -> StateController.setObjectiveState(ObjectiveState.SCORING)));

    operatorController.axisGreaterThan(3, -2.0/3.0)
      .and(operatorController.axisLessThan(3, 1.0/3.0))
      .and(isTeleop)
      .and(operatorController.getHID()::isConnected)
      .onTrue(new InstantCommand(() -> StateController.setObjectiveState(ObjectiveState.CLIMBING)));

    operatorController.axisGreaterThan(3, 1.0/3.0)
      .and(isTeleop)
      .and(operatorController.getHID()::isConnected)
      .onTrue(new InstantCommand(() -> StateController.setObjectiveState(ObjectiveState.DEFENDING)));
    
    // Reverse intake when robot is LOADED and SCORING
    /*StateController.buildTrigger(NoteState.LOADED, ObjectiveState.SCORING)
      .whileTrue(new IntakeCommand(intake, false));*/
  }

  public Command getAutonomousCommand() {
    return auto.getSelected();
  }

  // Periodically update dashboard
  public void periodic() {
    dashboard.periodic();
  }

  // Test modes
  public void testInit() { arm.initTestMode(); }
  public void testExec() { arm.execTestMode(); }
  public void testEnd() { arm.endTestMode(); }
}
