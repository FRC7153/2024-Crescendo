package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootingRegressions;
import frc.robot.subsystems.Arm;
import frc.robot.util.PVCamera;
import frc.robot.util.Util;


public class ArmToRegressionCommand extends Command {
  // Subsystems
  private Arm arm;
  private PVCamera camera;

  /**
   * Follows the regression with the distance, moving the arm.
   * @param arm
   * @param regression
   * @param distSupplier
   */
  public ArmToRegressionCommand(Arm arm, PVCamera camera) {
    this.arm = arm;
    this.camera = camera;
  }

  // Run
  @Override
  public void execute() {
    double dist = camera.getDistanceToTag(Util.isRedAlliance() ? 4 : 7);
    arm.setState(ShootingRegressions.V1_SHOOT_ANGLE_FROM_REAR_LIMELIGHT_DISTANCE(dist));
  }
}
