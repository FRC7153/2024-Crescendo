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
import frc.robot.auto.Autonomous;
import frc.robot.commands.ArmToStateCommand;
import frc.robot.commands.BalanceArmClimbCommand;
import frc.robot.commands.ClimberStageCommand;
import frc.robot.commands.IndexerRegripCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LoadShooterCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.ReverseIndexerCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.util.Dashboard;
import frc.robot.util.PVCamera;
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
  //private LED led = new LED(); // NOT instantiated

  // Cameras
  private PVCamera frontArmCamera = new PVCamera("Front Arm Camera");

  // Auto
  private Autonomous auto = new Autonomous(driveBase, arm, shooter, indexer);

  // Controls
  private CommandXboxController driverXboxController = new CommandXboxController(0);
  private CommandJoystick operatorController = new CommandJoystick(1);
  
  private Dashboard dashboard = new Dashboard(driveBase, frontArmCamera, auto);

  // Init
  public RobotContainer() {
    // Default
    driveBase.initDefaultCommand();
    arm.initDefaultCommand();
    shooter.initDefaultCommand();
    indexer.initDefaultCommand();
    intake.initDefaultCommand();
    //led.initDefaultCommand();
    
    // Config bindings
    configureBindings();

    // Config Auto
  }

  private void configureBindings() {
    // Teleop trigger (reused multiple times)
    Trigger isTeleop = new Trigger(DriverStation::isTeleopEnabled);

    // Drive Control
    isTeleop.whileTrue(new TeleopDriveCommand(
        driveBase, 
        () -> -driverXboxController.getLeftY(), 
        () -> driverXboxController.getLeftX(), 
        () -> -driverXboxController.getRightX()
      ).repeatedly());

    // Driver Intake Button (RT)
    driverXboxController.rightTrigger()
      .onTrue(
        new LoadShooterCommand(arm, shooter, intake, indexer, ArmPositions.kGROUND_INTAKE, true).until(driverXboxController.rightTrigger().negate())
        .andThen(intake::end, intake)
        .andThen(new IndexerRegripCommand(indexer))
      )
      .onTrue(new InstantCommand(frontArmCamera::snapshot));

    // Driver Source Intake Button (LT)
    driverXboxController.leftTrigger()
      .whileTrue(new PrintCommand("Source pickup not yet implemented")); // TODO SOURCE PICKUP

    // Driver Reverse Intake Button (Right Bumper)
    driverXboxController.rightBumper()
      .and(operatorController.trigger().negate()) // Not while trying to shoot!
      .whileTrue(new IntakeCommand(intake, false))
      .whileTrue(new ReverseIndexerCommand(indexer));
    
    // Operator Arm Speaker Button (6)
    operatorController.button(6)
      .whileTrue(new PrintCommand("Speaker long shot not yet implemented"));
      //.whileTrue(new ArmSpeakerCommand(arm, shooter, led, () -> driveBase.getPosition(false)));
      /*.whileTrue(new TeleopDriveHeadingLockCommand(
        driveBase, 
        () -> -driverXboxController.getLeftY(), 
        () -> -driverXboxController.getRightX()
      ));*/ // TODO driver should have control of heading lock

    // Operator Arm Amp Button (4)
    operatorController.button(4)
      .whileTrue(new ArmToStateCommand(arm, ArmPositions.kREAR_AMP, ArmPositions.kFRONT_AMP, driveBase::getAllianceOrientedYaw, 0.0, 180.0));

    // Operator Arm Preset Speaker Button (7)
    operatorController.button(7)
      .whileTrue(new ArmToStateCommand(arm, ArmPositions.kSUBWOOFER_SPEAKER_REAR, ArmPositions.kSUBWOOFER_SPEAKER_FRONT, driveBase::getYaw, 90.0, 270.0))
      .whileTrue(new InstantCommand(() -> shooter.setShootVelocity(3500.0), shooter).repeatedly());

    // Operator Shoot Button (trigger)
    // Assumes shooter is already up-to-speed (if needed)
    operatorController.trigger()
      .whileTrue(new ConditionalCommand(
        new ShootCommand(indexer, true), // Shoot forwards
        new ShootCommand(indexer, false), // Shoot backwards (if placing in AMP from rear)
        shooter::isShooterRunning
      ));
    
    // Operator Climb Button (6)
    operatorController.button(6)
      .onTrue(new ClimberStageCommand(climber, 67.0)); // 70.0

    // Operator Climb Button (4)
    operatorController.button(4)
      .onTrue(new ClimberStageCommand(climber, 0.0));

    // Operator Balance (throttle up)
    operatorController.axisLessThan(operatorController.getThrottleChannel(), 0.0)
      .whileTrue(new BalanceArmClimbCommand(arm));
  }

  public Command getAutonomousCommand() {
    return auto.getSelected();
  }

  // Periodically update dashboard
  public void periodic() {
    dashboard.periodic();
  }

  // Test modes
  public void testInit() { arm.initTestMode(); shooter.testInit(); indexer.stop(); }
  public void testExec() { arm.execTestMode(); shooter.testExec(); indexer.testExec(operatorController); }
  public void testEnd() { arm.endTestMode(); shooter.testEnd(); indexer.stop(); }
}
