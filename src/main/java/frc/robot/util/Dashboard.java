package frc.robot.util;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.auto.Autonomous;
import frc.robot.commands.dashboard.RecheckSwerveModulesCommand;
import frc.robot.commands.dashboard.ResetGyroCommand;
import frc.robot.commands.dashboard.ResetOdometryCommand;
import frc.robot.subsystems.drive.SwerveBase;

/**
 * Handles communicating with the driver with the dashboard (Shuffleboard)
 */
public class Dashboard {
  // Subsystems
  //private SwerveBase base;

  // Init
  public Dashboard(SwerveBase base, PVCamera camera, Autonomous auto) {
    // Remember subsystems
    //this.base = base;

    // Create tab
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    
    // Reset odometry button
    driveTab.add("Reset Odometry", new ResetOdometryCommand(base))
      .withSize(1, 1)
      .withPosition(3, 2);

    driveTab.add("Recheck Swerves", new RecheckSwerveModulesCommand(base))
      .withSize(1, 1)
      .withPosition(3, 1);

    // Add Auto chooser
    driveTab.add("Auto", auto.getChooser())
      .withSize(2, 1)
      .withPosition(3, 0);

    driveTab.add("Reset Gyro!!", new ResetGyroCommand(base))
      .withSize(1, 1)
      .withPosition(4, 1);

    // Add camera
    driveTab.addCamera("Limelight", "limelight", "http://limelight-aetos.local:5800/")
      .withSize(3, 4)
      .withPosition(0, 0)
      .withWidget(BuiltInWidgets.kCameraStream)
      .withProperties(Map.of("Show controls", false));
  }

  /** Call periodically to update outputs */
  public void periodic() {
    
  }
}
