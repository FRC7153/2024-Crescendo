package frc.robot.subsystems.drive;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;

import com.frc7153.diagnostics.DiagUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Wrapper for PhotonVision camera
 */
public class AprilTagCamera extends SubsystemBase {
    // Field layout
    private static AprilTagFieldLayout kAprilTagLayout;

    // Camera
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;

    private SwerveBase base;

    // Logging
    private DoubleLogEntry fpsLog;
    private DoubleLogEntry latencyLog;
    private DoubleLogEntry numTargetsLog;

    private double start = -1.0; // When results started arriving
    private double lastTimeStamp = -1.0; // Previous result's time stamp
    private int frameCount = 0; // Total number of frames

    /**
     * @param name NT name for camera
     * @param robotToCamera From center of robot to camera (standard WPI coordinate system)
     * @param base Reference to the swerve base
     */
    public AprilTagCamera(String name, Transform3d robotToCamera, SwerveBase base) {
        // Ensure a field layout exists
        if (kAprilTagLayout == null) {
            try {
                kAprilTagLayout = new AprilTagFieldLayout(DriveConstants.kAPRIL_TAG_LAYOUT_JSON);
            } catch (IOException e) {
                DiagUtil.criticalError("Failed to load AprilTagFieldLayout: %s", e.getMessage());
                e.printStackTrace();
            }
        }

        // Init camera
        camera = new PhotonCamera(name);
        this.base = base;

        camera.setLED(VisionLEDMode.kOff); // Ensure LEDs are off
        camera.setDriverMode(false); // Disable driver mode
        camera.setPipelineIndex(0); // Go to correct pipeline

        // Init pose estimator
        estimator = new PhotonPoseEstimator(kAprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);

        // Begin diagnostics
        DiagUtil.addDevice(camera);

        // Begin logs
        String logId = String.format("Vision/%s/", name);

        fpsLog = new DoubleLogEntry(DataLogManager.getLog(), logId + "FPS");
        latencyLog = new DoubleLogEntry(DataLogManager.getLog(), logId + "Latency");
        numTargetsLog = new DoubleLogEntry(DataLogManager.getLog(), logId + "Target Count");
    }

    // Periodic
    @Override
    public void periodic() {
        // Update
        Optional<EstimatedRobotPose> pose = estimator.update();

        // Check if new measurement is available
        if (pose.isPresent()) base.addVisionMeasurement(pose.get());

        // Recalculate log stats
        if (camera.getLatestResult().getTimestampSeconds() != lastTimeStamp) {
            lastTimeStamp = camera.getLatestResult().getTimestampSeconds();
            frameCount++;
            
            if (start == -1) {
                start = Timer.getFPGATimestamp();
            } else {
                fpsLog.append(frameCount/(Timer.getFPGATimestamp()-start));
            }

            latencyLog.append(camera.getLatestResult().getLatencyMillis());
            numTargetsLog.append(camera.getLatestResult().targets.size());
        }
    }
}
