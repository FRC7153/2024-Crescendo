package frc.robot.util;

import java.io.IOException;
import java.util.HashMap;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.frc7153.diagnostics.DiagUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.SwerveBase;

/**
 * PhotonVision Apriltag Camera
 */
public class PVCamera extends SubsystemBase {
  /** Class for storing PhotonTrackedTarget and timestamp */
  private static class TimestampedPhotonTrackedTarget {
    private PhotonTrackedTarget target = null;
    private double timestamp = -1.0;
  }

  // Shared field layout
  private static AprilTagFieldLayout kAprilTagLayout;

  // Camera
  private PhotonCamera camera;

  // Cached points of interest
  private HashMap<Integer, TimestampedPhotonTrackedTarget> tagCache = new HashMap<>(2);
  private double lastPacket = -1.0;

  // Estimator
  private PhotonPoseEstimator poseEstimator;
  private SwerveBase base;

  // Log
  private DoubleLogEntry latencyLog, distLog, angleLog;
  private IntegerLogEntry numTargetsLog;

  // Output (only tag 7)
  private GenericPublisher distOut, angleOut, ageOut;

  /**
   * Create new PhotonVision AprilTag Camera, without pose estimation
   * @param name
   */
  public PVCamera(String name) {
    this(name, null, null);
  }

  /**
   * Create new PhotonVision Apriltag Camera
   * @param name
   * @param base if not null, will do pose estimation on this camera
   * @param robotToCamera if not null, will start pose estimation on this camera
   */
  public PVCamera(String name, SwerveBase base, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);

    // Default configs
    camera.setDriverMode(false);
    camera.setPipelineIndex(0);
    camera.setLED(VisionLEDMode.kOff);

    // Default cache (speaker tags)
    //tagCache.put(3, new TimestampedPhotonTrackedTarget());
    tagCache.put(4, new TimestampedPhotonTrackedTarget());
    tagCache.put(7, new TimestampedPhotonTrackedTarget());
    //tagCache.put(8, new TimestampedPhotonTrackedTarget());

    // If estimator:
    if (robotToCamera != null && base != null) {
      this.base = base;

      if (kAprilTagLayout == null) {
        try {
          kAprilTagLayout = new AprilTagFieldLayout(DriveConstants.kAPRIL_TAG_LAYOUT_JSON);
        } catch (IOException e) {
          DiagUtil.criticalError("Failed to load AprilTagFieldLayout: %s", e.getMessage());
          e.printStackTrace();
          kAprilTagLayout = null;
        }
      }
    
      poseEstimator = new PhotonPoseEstimator(
        kAprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
    }
    
    // Log
    DiagUtil.addDevice(camera);

    latencyLog = new DoubleLogEntry(DataLogManager.getLog(), String.format("Vision/%s/latency", name), "ms");
    numTargetsLog = new IntegerLogEntry(DataLogManager.getLog(), String.format("Vision/%s/target count", name));

    distLog = new DoubleLogEntry(DataLogManager.getLog(), String.format("Vision/%s/distance", name), "m, tag 7");
    angleLog = new DoubleLogEntry(DataLogManager.getLog(), String.format("Vision/%s/angle", name), "deg, tag 7");

    // Output
    if (BuildConstants.kOUTPUT_ALL_TELEMETRY) {
      ShuffleboardTab tab = Shuffleboard.getTab(String.format("Camera - %s", name));

      distOut = tab.add("Distance (7) (m)", -1.0)
        .getEntry().getTopic().genericPublish("double");

      angleOut = tab.add("Angle (7) (deg)", -999.0)
        .getEntry().getTopic().genericPublish("double");
        
      ageOut = tab.add("Age (7) (s)", -1.0)
        .getEntry().getTopic().genericPublish("double");
    }

    // Begin tracking
    register();
  }

  // Fill cache
  @Override
  public void periodic() {
    try { // getLatestResult() throws!
        if (!camera.isConnected()) return; // Camera not up yet
        PhotonPipelineResult results = camera.getLatestResult();

        if (results.getTimestampSeconds() == lastPacket) return; // No new packets
        lastPacket = results.getTimestampSeconds();
        
        // Get pose update
        if (poseEstimator != null && base != null && kAprilTagLayout != null) {
          Optional<EstimatedRobotPose> pose = poseEstimator.update();
          if (pose.isPresent()) base.addVisionMeasurement(pose.get());
        }

        // Cache new results
        for (PhotonTrackedTarget target : results.targets) {
          if (tagCache.containsKey(target.getFiducialId())) {
            // This is a cached tag
            tagCache.get(target.getFiducialId()).target = target;
            tagCache.get(target.getFiducialId()).timestamp = Timer.getFPGATimestamp();

            // Output?
            if (target.getFiducialId() == 7 && !Util.isRedAlliance()) {
              if (BuildConstants.kOUTPUT_ALL_TELEMETRY) {
                distOut.setDouble(getDistanceToSpeaker());
                angleOut.setDouble(getAngleToSpeaker());
                ageOut.setDouble(getTagCacheAge(7));
              }

              distLog.append(getDistanceToSpeaker());
              angleLog.append(getAngleToSpeaker());
            }
          }
        }

      // Log
      numTargetsLog.append(results.targets.size());
      latencyLog.append(results.getLatencyMillis());
    } catch (Exception e) {
      DriverStation.reportWarning(
        String.format("PhotonVision Camera '%s' refresh has thrown: ", camera.getName(), e.getMessage()), 
        false
      );
    }
  }

  /**
   * Gets the distance to the cached speaker tag
   * @return meters
   */
  public double getDistanceToSpeaker() {
    TimestampedPhotonTrackedTarget speakerTag = tagCache.get((Util.isRedAlliance()) ? 4 : 7);

    if (speakerTag.target != null) {
      Transform3d proj = speakerTag.target.getBestCameraToTarget();
      return proj.getTranslation().getNorm();
    } else {
      DriverStation.reportWarning("No tags seen yet!", true);
      return 2.0;
    }
  }

  /**
   * Gets the angle to the cached speaker tag.
   * @return deg, CCW+
   */
  public double getAngleToSpeaker() {
    TimestampedPhotonTrackedTarget speakerTag = tagCache.get((Util.isRedAlliance()) ? 4 : 7);

    if (speakerTag.target != null) {
      return speakerTag.target.getYaw();
    } else {
      DriverStation.reportWarning("No tags seen yet!", true);
      return 0.0;
    }
  }

  /**
   * Age of tag in cache
   * @return seconds
   */
  public double getTagCacheAge(int tag) {
    return Timer.getFPGATimestamp() - tagCache.get(tag).timestamp;
  }

  /**
   * Take picture
   */
  public void snapshot() {
    System.out.println("Snap!");
    camera.takeOutputSnapshot();
  }
}
