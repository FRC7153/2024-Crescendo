package frc.robot.util;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.dashboard.ResetOdometryCommand;
import frc.robot.subsystems.drive.SwerveBase;

/**
 * Handles communicating with the driver with the dashboard (Shuffleboard)
 */
public class Dashboard {
  // Subsystems
  private SwerveBase base;

  // Outputs
  private GenericEntry globalPoseOutput, relativePoseOutput;

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
  }

  /** Call periodically to update outputs */
  public void periodic() {
    // Update pose estimation
    globalPoseOutput.setString(base.getPosition(true).toString());
    relativePoseOutput.setString(base.getPosition(false).toString());
  }
}
