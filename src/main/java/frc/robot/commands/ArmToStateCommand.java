package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmState;

public class ArmToStateCommand extends Command {
  // Subsystems
  private Arm arm;
  private ArmState target;

  // Constructor
  /**
   * Never-ending command that sets the Arm's position.
   * Will be canceled if interrupted.
   */
  public ArmToStateCommand(Arm arm, ArmState target) {
    this.arm = arm;
    this.target = target;
    
    addRequirements(arm);
  }

  // Init
  @Override
  public void initialize() {
    arm.setState(target);
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
}
