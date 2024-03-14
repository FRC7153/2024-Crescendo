package frc.robot.util;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.commands.dashboard.ResetOdometryCommand;
import frc.robot.subsystems.drive.SwerveBase;

/**
 * Handles communicating with the driver with the dashboard (Shuffleboard)
 */
public class Dashboard {
  // Subsystems
  private SwerveBase base;
  PhotonCamera camera = new PhotonCamera("Secondarypi");

  // Outputs
  private GenericEntry globalPoseOutput, relativePoseOutput;
  private GenericEntry cameraStream;

  // Init
  public Dashboard(SwerveBase base) {
    // Remember subsystems
    this.base = base;

    // Create tab
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    
    // Reset odometry button
    driveTab.add("Reset Odometry", new ResetOdometryCommand(base))
      .withPosition(3, 0);

    // Pose estimation
    globalPoseOutput = driveTab.add("Odometry Pose (global)", "?")
      .withPosition(0, 0)
      .withSize(3, 1)
      .getEntry();

    relativePoseOutput = driveTab.add("Odometry Pose (relative)", "?")
      .withPosition(0, 1)
      .withSize(3, 1)
      .getEntry();

    cameraStream = driveTab.add("Camera Stream", "?")
    .withPosition(5, 5)
    .withSize(3,3)
    .getEntry();
  }

  /** Call periodically to update outputs */
  public void periodic() {
    // Update pose estimation
    globalPoseOutput.setString(base.getPosition(true).toString());
    relativePoseOutput.setString(base.getPosition(false).toString());
  }
}
