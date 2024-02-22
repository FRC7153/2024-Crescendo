package frc.robot.commands.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.SwerveBase;

public class ResetOdometryCommand extends InstantCommand {

  public ResetOdometryCommand(SwerveBase base) {
    super(() -> base.resetPosition(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0))));
  }

  @Override
  public boolean runsWhenDisabled() { return true; }

  @Override
  public String getName() { return "Reset odometry (0, 0)"; }
}
