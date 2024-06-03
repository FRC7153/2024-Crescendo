package frc.robot.commands;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShootingRegressions;
import frc.robot.subsystems.Arm;
import frc.robot.util.LimelightCamera;


public class ArmToRegressionCommand extends Command {
  // Subsystems
  private Arm arm;
  private LimelightCamera camera;

  private LinearFilter filter = LinearFilter.movingAverage(4);
  private int filterInputs = 0;

  /**
   * Follows the regression with the distance, moving the arm.
   * @param arm
   * @param distSupplier
   */
  public ArmToRegressionCommand(Arm arm, LimelightCamera camera) {
    this.arm = arm;
    this.camera = camera;

    addRequirements(arm);
  }

  // Run
  @Override
  public void initialize() {
    arm.setExtension(0.0);
    arm.setLowerPivotAngle(133.0);
    arm.setUpperPivotAngle(180.0);

    filter.reset();
    filterInputs = 0;
  }

  @Override
  public void execute() {
    double dist = camera.getDistanceToTag();
    dist = MathUtil.clamp(dist, 2.10, 4.0);

    double last = filter.calculate(dist);
    filterInputs++;

    if (filterInputs < 4) {
      arm.setUpperPivotAngle(ShootingRegressions.LIMELIGHT_REGRESSION_V3(dist));
    } else {
      arm.setUpperPivotAngle(ShootingRegressions.LIMELIGHT_REGRESSION_V3(last));
    }
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

  @Override
  public String getName() {
    return "ArmRegressionCommand";
  }
}
