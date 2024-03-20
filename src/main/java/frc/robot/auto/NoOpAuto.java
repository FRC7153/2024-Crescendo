package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.SwerveBase;

public class NoOpAuto extends SequentialCommandGroup {
  /**
   * Does nothing, except set the estimator's position in auto.
   * @param base
   */
  public NoOpAuto(SwerveBase base) {
    super(
      new PrintCommand("Setting auto position..."),
      new InstantCommand(() -> base.resetPosition(AutoUtils.defaultInitPos())),
      new PrintCommand("No-Op Auto: pos set and waiting...")
    );
  }
}
