package frc.robot.commands.dashboard;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.SwerveBase;

public class ResetGyroCommand extends InstantCommand {

  public ResetGyroCommand(SwerveBase base) {
    super(() -> {
      System.out.printf("Prior to gyro reset, yaw is %fdeg\n", base.getYaw().getDegrees());
      base.setYawAngle(0.0);
      System.out.println("Reset gyro to 0.0!");
    });
  }

  @Override
  public boolean runsWhenDisabled() { return true; }

  @Override
  public String getName() { return "Reset yaw (0)"; }
}
