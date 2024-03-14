package frc.robot.util;

import java.util.Map;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.auto.Autonomous;
import frc.robot.commands.dashboard.ResetOdometryCommand;
import frc.robot.subsystems.drive.SwerveBase;

/**
 * Handles communicating with the driver with the dashboard (Shuffleboard)
 */
public class Dashboard {
  // Subsystems
  private SwerveBase base;

  // Outputs

  // Init
  public Dashboard(SwerveBase base, PVCamera camera, Autonomous auto) {
    // Remember subsystems
    this.base = base;

    // Create tab
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    
    // Reset odometry button
    driveTab.add("Reset Odometry", new ResetOdometryCommand(base))
      .withPosition(0, 3);

    // Add Auto chooser
    driveTab.add("Auto", auto.getChooser())
      .withPosition(1, 3)
      .withSize(2, 1);

    // TODO camera name here
    /*driveTab.add("Front Arm Camera", CameraServer.getServer("Front Arm Camera").getSource())
      .withWidget(BuiltInWidgets.kCameraStream)
      .withPosition(0, 0)
      .withSize(3, 4)
      .withProperties(Map.of("SHOW CONTROLS", false));*/
  }

  /** Call periodically to update outputs */
  public void periodic() {
    
  }
}
