package frc.robot.util;

import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.frc7153.diagnostics.DiagUtil;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * PhotonVision Apriltag Camera
 */
public class PVCamera extends SubsystemBase {
  // Camera
  private PhotonCamera camera;

  // Cached points of interest
  private HashMap<Integer, PhotonTrackedTarget> tagCache = new HashMap<>(4);
  private double lastPacket = -1.0;

  /**
   * Create new PhotonVision Apriltag Camera
   * @param name
   */
  public PVCamera(String name) {
    camera = new PhotonCamera(name);

    // Default configs
    camera.setDriverMode(false);
    camera.setPipelineIndex(0);
    camera.setLED(VisionLEDMode.kOff);

    // Default cache
    tagCache.put(3, null);
    tagCache.put(4, null);
    tagCache.put(7, null);
    tagCache.put(8, null);
    
    // Log
    DiagUtil.addDevice(camera);

    // Begin tracking
    register();
  }

  // Fill cache
  @Override
  public void periodic() {
    PhotonPipelineResult results = camera.getLatestResult();

    if (results.getTimestampSeconds() == lastPacket) return; // No new packets
    lastPacket = results.getTimestampSeconds();
    
    // Look at new results
    for (PhotonTrackedTarget target : results.targets) {
      if (tagCache.containsKey(target.getFiducialId())) {
        // This is a cached tag
        tagCache.put(target.getFiducialId(), target);
      }
    }
  }

  /**
   * Gets the distance to the tag
   * @return
   */
  public double getDistanceToSpeaker() {
    return 0.0;
  }
}
