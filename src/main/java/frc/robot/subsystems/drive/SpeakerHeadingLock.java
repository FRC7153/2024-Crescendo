package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.LimelightCamera;

public class SpeakerHeadingLock implements Supplier<Double> {
  // Pose supplier
  private Supplier<Pose2d> poseSupplier;

  // Cameras
  private LimelightCamera camera;

  // Control
  private PIDController headingPID = new PIDController(DriveConstants.kHEADING_CORRECTION_P, 0.0, 0.0);

  /**
   * Drives the heading of the robot to the speaker.
   * {@code .get()} value is in rot/s, not percentage
   * @param camera
   * @param poseSupplier the blue alliance-oriented pose
   */
  public SpeakerHeadingLock(LimelightCamera camera, Supplier<Pose2d> poseSupplier) {
    this.camera = camera;
    this.poseSupplier = poseSupplier;

    // Finish config PIDController
    headingPID.setSetpoint(0.0);
  }

  /**
   * Get the recommended rot/s to line up to speaker
   */
  @Override
  public Double get() {
    int tag = camera.getTagId();
    double yaw = camera.getTagYaw();

    if (tag == -1) {
      // We have no targets!
      Pose2d poseGuess = poseSupplier.get();

      double angle = FieldConstants.kSPEAKER_POS.minus(poseGuess.getTranslation()).getAngle().getDegrees();

      if (angle > 0.0) return 600.0;
      else return -600.0;
    } else {
      // We have target
      // TODO clamp this for safety
      return headingPID.calculate(yaw);
    }
  }
  
}
