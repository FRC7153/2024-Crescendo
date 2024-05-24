package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootingRegressions;
import frc.robot.subsystems.Arm;
import frc.robot.util.LimelightCamera;


public class ArmToRegressionCommand extends Command {
  // Subsystems
  private Arm arm;
  private LimelightCamera camera;

  /**
   * Follows the regression with the distance, moving the arm.
   * @param arm
   * @param distSupplier
   */
  public ArmToRegressionCommand(Arm arm, LimelightCamera camera) {
    this.arm = arm;
    this.camera = camera;
  }

  // Run
  @Override
  public void initialize() {
    arm.setExtension(0.0);
    arm.setLowerPivotAngle(125.0);
    arm.setUpperPivotAngle(180.0);
  }

  @Override
  public void execute() {
    double dist = camera.getDistanceToTag();

    // TODO safety clamp this
    dist = MathUtil.clamp(dist, 0.0, 20.0);

    arm.setUpperPivotAngle(ShootingRegressions.LIMELIGHT_REGRESSION_V3(dist));
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Cancel self
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelSelf;
  }
}
