package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.dashboard.ResetOdometryCommand;
import frc.robot.subsystems.drive.SwerveBase;

/**
 * Handles communicating with the driver with the dashboard (Shuffleboard)
 */
public class Dashboard {


  // Init
  public Dashboard(SwerveBase base) {
    // Create tab
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");
    
    // Reset odometry button
    driveTab.add("Reset Odometry", new ResetOdometryCommand(base))
      .withPosition(0, 0);
  }
}
