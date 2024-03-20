package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.util.PVCamera;
import frc.robot.util.Util;

public class SpeakerHeadingLock implements Supplier<Double> {
  // Pose supplier
  private Supplier<Pose2d> poseSupplier;

  // Cameras
  private PVCamera rearCamera;

  // Control
  private PIDController headingPID = new PIDController(
    DriveConstants.kHEADING_CORRECTION_P, 
    DriveConstants.kHEADING_CORRECTION_I,
    DriveConstants.kHEADING_CORRECTION_D
  );

  /**
   * Drives the heading of the robot to the speaker.
   * {@code .get()} value is in rot/s, not percentage
   * @param rearCamera
   * @param poseSupplier the alliance-oriented pose
   */
  public SpeakerHeadingLock(PVCamera rearCamera, Supplier<Pose2d> poseSupplier) {
    this.rearCamera = rearCamera;
    this.poseSupplier = poseSupplier;

    // Finish config PIDController
    headingPID.setSetpoint(0.0);
  }

  /**
   * Get the recommended rot/s to line up to speaker
   */
  @Override
  public Double get() {
    int targetTag = (Util.isRedAlliance()) ? 4 : 7;

    // Once we have two cameras, we will see which one is newer here
    PVCamera bestCam = this.rearCamera;

    if (bestCam.getTagCacheTime(targetTag) == -1.0) {
      // This tag was never seen, take best guess!
      // With two cameras, pose supplier can better be used to take a better guess
      return 250.0;
    }

    return MathUtil.clamp(
      headingPID.calculate(bestCam.getAngleToTag(targetTag)),
      -600.0,
      600.0
    );
  }
  
}
