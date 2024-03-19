package com.frc7153.controllers;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frc7153.diagnostics.DiagUtil;

import edu.wpi.first.math.MathUtil;

/**
 * To prevent possible issue where CANCoder did not boot in time to apply configurations or provide
 * meaningful values.
 * All magnet offset math is done on the Rio, not the encoder.
 */
public class SafeCANCoder {
  // Encoder and local configs
  private CANcoder encoder;
  private MagnetSensorConfigs localConfigs = new MagnetSensorConfigs();

  // Have we received any signal yet?
  private boolean hasSeenUpdates = false;

  /**
   * Create new SafeCANCoder
   * @param canID CAN Id
   * @param busName CAN bus name ("rio" for RoboRio's bus)
   */
  public SafeCANCoder(int canID, String busName) {
    encoder = new CANcoder(canID, busName);
    
    // Reset all configs (should already be done, just to be safe)
    CANcoderConfiguration config = new CANcoderConfiguration();
    encoder.getConfigurator().apply(config);
  }

  /**
   * Create a new SafeCANCoder
   * @param canID CAN Id
   */
  public SafeCANCoder(int canID) {
    this(canID, "rio");
  }

  /**
   * Sets the range of the absolute encoder (applied locally)
   * @param value
   */
  public void setRange(AbsoluteSensorRangeValue value) {
    localConfigs.AbsoluteSensorRange = value;
  }

  /**
   * Offset added to reported position (applied locally)
   * @param offset rotations
   */
  public void setMagnetOffset(double offset) {
    localConfigs.MagnetOffset = offset;
  }

  /**
   * Sets the sensor's direction, facing the LED side of the CANCoder
   * (applied locally)
   * @param direction
   */
  public void setMagnetDirection(SensorDirectionValue direction) {
    localConfigs.SensorDirection = direction;
  }

  /**
   * The position, with configs applied.
   * @return rotations
   */
  public double getAbsolutePosition() {
    // Check if updated yet
    boolean updated = encoder.getAbsolutePosition().hasUpdated();

    if (!hasSeenUpdates && !updated) {
      System.out.printf("Position of CANCoder %d checked without any updates received!\n", encoder.getDeviceID());
    } else if (!hasSeenUpdates && updated) {
      hasSeenUpdates = true;
    }

    // Get position
    double pos = encoder.getAbsolutePosition().getValue();

    // Invert
    if (localConfigs.SensorDirection.equals(SensorDirectionValue.Clockwise_Positive)) pos = -pos;
    
    // Apply offset
    pos += localConfigs.MagnetOffset;

    // Limit
    if (localConfigs.AbsoluteSensorRange.equals(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)) {
      pos = MathUtil.inputModulus(pos, -0.5, 0.5);
      if (pos == 0.5) pos = -0.5; // edge case
    } else {
      pos = MathUtil.inputModulus(pos, 0.0, 1.0);
      if (pos == 1.0) pos = 0.0; // edge case
    }

    return pos;
  }

  /**
   * The velocity, with configs applied
   * @return rotations/second
   */
  public double getVelocity() {
    // Check if updated yet
    boolean updated = encoder.getVelocity().hasUpdated();

    if (!hasSeenUpdates && !updated) {
      System.out.printf("Velocity of CANCoder %d checked without any updates received!\n", encoder.getDeviceID());
    } else if (!hasSeenUpdates && updated) {
      hasSeenUpdates = true;
    }

    // Get velocity
    double velo = encoder.getVelocity().getValue();

    // Invert
    if (localConfigs.SensorDirection.equals(SensorDirectionValue.Clockwise_Positive)) velo = -velo;

    return velo;
  }

  /**
   * Starts diagnostics on this device
   */
  public void initDiagnostics() {
    DiagUtil.addDevice(encoder);
  }
}
