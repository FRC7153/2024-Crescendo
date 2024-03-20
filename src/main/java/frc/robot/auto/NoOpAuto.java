package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.util.Util;

public class NoOpAuto extends SequentialCommandGroup {
  /**
   * Does nothing, except set the estimator's position in auto.
   * @param base
   */
  public NoOpAuto(SwerveBase base) {
    super(
      new PrintCommand("Setting auto position..."),
      new InstantCommand(() -> setDefaultPos(base)),
      new PrintCommand("No-Op Auto: pos set and waiting...")
    );
  }

  /**
   * Sets the base estimator's pos
   * @param base
   */
  private static void setDefaultPos(SwerveBase base) {
    base.resetPosition(new Pose2d(
      (Util.isRedAlliance()) ? FieldConstants.kFIELD_SIZE.getX() : 0.0,
      FieldConstants.kFIELD_SIZE.getY() / 2.0,
      Rotation2d.fromDegrees((Util.isRedAlliance()) ? 180.0 : 0.0)
    ));
  }
}
