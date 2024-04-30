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
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShootingRegressions;
import frc.robot.auto.Autonomous;
import frc.robot.commands.ArmSourceCommand;
import frc.robot.commands.ArmToRegressionCommand;
import frc.robot.commands.ArmToStateCommand;
import frc.robot.commands.BalanceArmClimbCommand;
import frc.robot.commands.ClimbBalancedCommand;
import frc.robot.commands.ClimberStageCommand;
import frc.robot.commands.IndexerRegripCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LoadShooterGroundCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.ReverseIndexerCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SpeakerHeadingLock;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.util.Dashboard;
import frc.robot.util.PDHLogger;
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

  // Cameras + Devices
  private PDHLogger pdh = new PDHLogger(HardwareConstants.kPDH_CAN);
  //private PVCamera frontArmCamera = new PVCamera("FrontUSBCamera");
  private PVCamera rearLLCamera = new PVCamera("RearLLCamera");

  // Auto
  private Autonomous auto = new Autonomous(driveBase, intake, arm, shooter, indexer);

  // Controls
  private CommandXboxController driverXboxController = new CommandXboxController(0);
  private CommandJoystick operatorController = new CommandJoystick(1);
  
  private Dashboard dashboard = new Dashboard(driveBase, rearLLCamera, auto);

  // Init
  public RobotContainer() {
    // Clear our sticky faults
    pdh.clearStickyFaults();

    // Default
    driveBase.initDefaultCommand();
    arm.initDefaultCommand();
    shooter.initDefaultCommand();
    indexer.initDefaultCommand();
    intake.initDefaultCommand();
    //led.initDefaultCommand();
    
    // Config bindings
    configureBindings();
  }

  private void configureBindings() {
    // Teleop trigger (reused multiple times)
    Trigger isTeleop = new Trigger(DriverStation::isTeleopEnabled);

    // Drive Control (Left joystick hold for fast mode) (while A is not held)
    driverXboxController.a().negate()
    .and(isTeleop)
      .whileTrue(new TeleopDriveCommand(
        driveBase, 
        () -> -driverXboxController.getLeftY(), 
        () -> driverXboxController.getLeftX(), 
        () -> -driverXboxController.getRightX(),
        true,
        () -> driverXboxController.leftStick().getAsBoolean(),
        () -> driverXboxController.leftTrigger().getAsBoolean()
      ).repeatedly());

    // Driver speaker heading lock (A held)
    driverXboxController.a()
      .whileTrue(new TeleopDriveCommand(
        driveBase, 
        () -> -driverXboxController.getLeftY(), 
        ()-> driverXboxController.getLeftX(),
        (new SpeakerHeadingLock(
          rearLLCamera, 
          () -> driveBase.getPosition(false)
        )),
        false,
        () -> false, // Don't allow fast mode here
        () -> false // Dont obstacle avoidance here
      ));

    // Driver Intake Button (RT)
    driverXboxController.rightTrigger()
      .onTrue(
        new LoadShooterGroundCommand(arm, shooter, intake, indexer).until(driverXboxController.rightTrigger().negate())
        .andThen(intake::end, intake)
        .andThen(new IndexerRegripCommand(indexer))
      );

    // Driver Reverse Intake Button (Right Bumper)
    driverXboxController.rightBumper()
      .and(operatorController.trigger().negate()) // Not while trying to shoot!
      .whileTrue(new IntakeCommand(intake, false))
      .whileTrue(new ReverseIndexerCommand(indexer));

    // Operator Source Intake Button (Button 2)
    operatorController.button(2)
      .onTrue(new ArmSourceCommand(arm, shooter, indexer, operatorController.button(2)));
    
    // Operator Arm Speaker Long Shot Button (6)
    operatorController.button(6)
      .and(driverXboxController.rightTrigger().negate())
      .whileTrue(new ArmToRegressionCommand(arm, rearLLCamera)) // TODO fix arm return issue
      .whileTrue(new InstantCommand(() -> shooter.setShootVelocity(2800.0), shooter).repeatedly()); // 3500

    // Operator Arm Amp Button (4)
    operatorController.button(4)
      .whileTrue(new ArmToStateCommand(arm, ArmPositions.kFRONT_AMP, ArmPositions.kREAR_AMP, driveBase::getReorientedYaw, 0.0, 180.0));

    // Operator Arm Preset Speaker Button (7)
    operatorController.button(7)
      .and(driverXboxController.rightTrigger().negate()) // Not while intaking
      .whileTrue(new ArmToStateCommand(arm, ArmPositions.kSUBWOOFER_SPEAKER_FRONT, ArmPositions.kSUBWOOFER_SPEAKER_REAR, driveBase::getYaw, 90.0, 270.0))
      .whileTrue(new InstantCommand(() -> shooter.setShootVelocity(3500.0), shooter).repeatedly()); // I know, this velocity is obscenely high. Don't question it :)

    // Operator Shoot Button (trigger)
    // Assumes shooter is already up-to-speed (if needed)
    operatorController.trigger()
      .whileTrue(new ConditionalCommand(
        new ShootCommand(indexer, true, () -> operatorController.button(4).getAsBoolean()), // Shoot forwards
        new ShootCommand(indexer, false, () -> operatorController.button(4).getAsBoolean()), // Shoot backwards (if placing in AMP from rear)
        shooter::isShooterRunning
      ));
    
    // Operator Climb Button (5)
    operatorController.button(5)
      .onTrue(new ClimberStageCommand(climber, 70.0)); // 70.0

    // Operator Climb Button (3)
    operatorController.button(3)
      .onTrue(new ClimberStageCommand(climber, 0.0));
      //.onTrue(new ClimbBalancedCommand(driveBase, climber));

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

  /**
   * Realigns the swerve relative encoders with the absolute encoders. Call at start of teleop/auto.
   */
  public void recheckSwerveHeadings() { driveBase.doubleCheckHeadings(); }

  /**
   * Restarts the timers for the arm's motion profile. Call at start of teleop.
   */
  public void resetArmMotionProfile() { arm.resetLowerPivotProfileTimer(); }

  // Test modes
  public void testInit() { arm.initTestMode(); shooter.testInit(); indexer.stop(); }
  public void testExec() { arm.execTestMode(); shooter.testExec(); indexer.testExec(operatorController); }
  public void testEnd() { arm.endTestMode(); shooter.testEnd(); indexer.stop(); }
}
