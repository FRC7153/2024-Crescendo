package frc.robot.util;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Some widely used util functions
 */
public class Util {
  /**
   * More resource-friendly way of checking whether the robot is red alliance
   * @return
   */
  public static boolean isRedAlliance() {
    AllianceStationID id = DriverStation.getRawAllianceStation();
    if (id.equals(AllianceStationID.Unknown)) DriverStation.reportWarning("Driver station alliance is unknown!", false);
    return (id.equals(AllianceStationID.Red1) || id.equals(AllianceStationID.Red2) || id.equals(AllianceStationID.Red3));
  }
}
