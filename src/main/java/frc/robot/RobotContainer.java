// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
import frc.robot.commands.TeleopDriveHeadingLockCommand;
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
  private Dashboard dashboard = new Dashboard(driveBase);

  public RobotContainer() {
    // Default
    driveBase.initDefaultCommand();
    arm.initDefaultCommand();
    shooter.initDefaultCommand();
    indexer.initDefaultCommand();
    led.initDefaultCommand();
    
    configureBindings();
  }

  private void configureBindings() {
    // Teleop trigger (reused multiple times)
    Trigger isTeleop = new Trigger(DriverStation::isTeleopEnabled);

    // Drive Control
    isTeleop.whileTrue(new TeleopDriveCommand(
        driveBase, 
        driverXboxController::getLeftY, 
        driverXboxController::getLeftX, 
        driverXboxController::getRightX
      ).repeatedly());
      
    // Operator Arm Speaker Button (6) pressed while robot is LOADED and SCORING
    operatorController.button(6).and(StateController.buildTrigger(NoteState.LOADED, ObjectiveState.SCORING))
      .whileTrue(new ArmSpeakerCommand(arm, shooter, led, () -> driveBase.getPosition(false)))
      .whileTrue(new TeleopDriveHeadingLockCommand(
        driveBase, 
        driverXboxController::getLeftY, 
        driverXboxController::getRightX
      ));

    // Operator Arm Amp Button (4) pressed while robot is LOADED and SCORING
    operatorController.button(4).and(StateController.buildTrigger(NoteState.LOADED, ObjectiveState.SCORING))
      .whileTrue(new ArmAmpCommand(arm, led, () -> driveBase.getPosition(false)));

    // Operator Source Button (2) pressed while robot is NOT LOADED and SCORING
    operatorController.button(2).and(StateController.buildTrigger(NoteState.EMPTY, ObjectiveState.SCORING))
      .whileTrue(new InstantCommand()); // TODO SOURCE PICKUP

    // Operator Shoot Button (trigger) pressed while robot is LOADED and SCORING
    operatorController.trigger().and(StateController.buildTrigger(NoteState.LOADED, ObjectiveState.SCORING))
      .whileTrue(new ShootCommand(indexer, true)); // Will change NoteState to EMPTY
    
    // Operator Climb Button (6) pressed while robot is CLIMBING
    operatorController.button(6).and(StateController.buildTrigger(null, ObjectiveState.CLIMBING))
      .onTrue(new ClimberStageCommand(climber, true));

    // Operator Climb Button (4) pressed while robot is CLIMBING
    operatorController.button(4).and(StateController.buildTrigger(null, ObjectiveState.CLIMBING))
      .onTrue(new ClimberStageCommand(climber, false));
    
    // Handle Objective State Control (Operator throttle)
    operatorController.axisLessThan(Joystick.AxisType.kThrottle.value, -2.0/3.0)
      .and(isTeleop)
      .onTrue(new InstantCommand(() -> StateController.setObjectiveState(ObjectiveState.SCORING)));

    operatorController.axisGreaterThan(Joystick.AxisType.kThrottle.value, -2.0/3.0)
      .and(operatorController.axisLessThan(Joystick.AxisType.kThrottle.value, 1.0/3.0))
      .and(isTeleop)
      .onTrue(new InstantCommand(() -> StateController.setObjectiveState(ObjectiveState.CLIMBING)));

    operatorController.axisGreaterThan(Joystick.AxisType.kThrottle.value, 1.0/3.0)
      .and(isTeleop)
      .onTrue(new InstantCommand(() -> StateController.setObjectiveState(ObjectiveState.DEFENDING)));
    
    // Don't intake when robot is LOADED and SCORING
    StateController.buildTrigger(NoteState.LOADED, ObjectiveState.SCORING)
      .whileTrue(new IntakeCommand(intake, false));

    // Intake when robot is EMPTY/PROCESSING and SCORING
    StateController.buildTrigger(NoteState.EMPTY, ObjectiveState.SCORING)
      .or(StateController.buildTrigger(NoteState.PROCESSING, ObjectiveState.SCORING))
      .whileTrue(new IntakeCommand(intake, true)) // Will change NoteState to PROCESSING
      .whileTrue(new LoadShooterCommand(shooter, indexer, led)); // Will change NoteState to LOADED
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
