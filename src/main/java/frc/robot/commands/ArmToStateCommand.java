package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;

public class ArmToStateCommand extends Command {
  // Subsystems
  private Arm arm;
  private ArmState targetForward, targetBackward;
  private Supplier<Rotation2d> rotSupplier;
  private double minAngle, maxAngle;
  private boolean highPriority;

  // Constructor
  /**
   * Never-ending command that sets the Arm's position.
   * Will be canceled if interrupted.
   */
  public ArmToStateCommand(Arm arm, ArmState target) {
    this(arm, target, null, null, 0.0, 0.0);
  }

  /**
   * Never-ending command that sets the Arm's position, depending on the heading of the robot.
   * If the heading [0.0, 360.0) is between [minAngle, maxAngle), it will use the forward target, else
   * it will use the backward target. 
   * This is an overloaded constructor, if you only need one state use the other one.
   * @param arm
   * @param forwardTarget
   * @param backwardTarget
   * @param poseSupplier
   * @param minAngle
   * @param maxAngle
   * @param highPriority 
   */
  public ArmToStateCommand(
    Arm arm, 
    ArmState forwardTarget, 
    ArmState backwardTarget, 
    Supplier<Rotation2d> rotSupplier,
    double minAngle, 
    double maxAngle//,
    //boolean highPriority
  ) {
    this.arm = arm;
    this.targetForward = forwardTarget;
    this.targetBackward = backwardTarget;
    this.rotSupplier = rotSupplier;
    this.minAngle = minAngle;
    this.maxAngle = maxAngle;
    this.highPriority = highPriority; // TODO

    addRequirements(arm);
  }

  // Init
  @Override
  public void initialize() {
    if (targetBackward == null) {
      // Only one state
      arm.setState(targetForward);
    }
  }

  // Run
  @Override
  public void execute() {
    if (targetBackward == null) return; // Only one state

    double heading = MathUtil.inputModulus(rotSupplier.get().getDegrees(), 0.0, 360.0);

    if (heading >= minAngle && heading < maxAngle) {
      // Forward target
      arm.setState(targetForward);
    } else {
      // Backward target
      arm.setState(targetBackward);
    }
  }

  // Don't end
  @Override
  public boolean isFinished() {
    return false;
  }

  // Cancel self
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelSelf;
  }

  // Format name
  @Override
  public String getName() {
    if (targetBackward == null) return String.format("ArmToStateCommand: %s", targetForward);
    else {
      return String.format("ArmToStateCommand: %s and %s", targetForward, targetBackward);
    }
  }
}
