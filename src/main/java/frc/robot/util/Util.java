package frc.robot.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

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

  /**
   * Decreases the CAN utilization for the specified SparkMax by disabling the encoder frames.
   * @param spark
   */
  public static void disableExternalEncoderFrames(CANSparkMax spark) {
    // 60000ms = 1 minutes
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 60000); // Analog sensor
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 60000); // Alt encoder
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 60000); // Duty cycle encoder position
    spark.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 60000); // Duty cycle encoder velocity
  }
}