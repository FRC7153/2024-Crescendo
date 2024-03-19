package frc.robot.commands.dashboard;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.SwerveBase;

public class RecheckSwerveModulesCommand extends InstantCommand {
  /**
   * Rechecks the heading of all the swerve modules.
   * Does not require anything.
   * @param base
   */
  public RecheckSwerveModulesCommand(SwerveBase base) {
    super(() -> base.doubleCheckHeadings());
  }

  @Override
  public boolean runsWhenDisabled() { return true; }

  @Override
  public String getName() { return "Recheck Swerve Modules"; }
}
