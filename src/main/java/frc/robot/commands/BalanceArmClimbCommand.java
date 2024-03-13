package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.Arm;

public class BalanceArmClimbCommand extends Command {
  // Subsystems
  private Arm arm;

  // Constructor
  public BalanceArmClimbCommand(Arm arm) {
    this.arm = arm;
  }

  // Init
  @Override
  public void initialize() {
    arm.setState(ArmPositions.kCLIMB_BALANCE);
  }

  // End
  @Override
  public void end(boolean terminated) {
    arm.setState(ArmPositions.kDEFAULT);
  }

  // Never end
  @Override
  public boolean isFinished() {
    return false;
  }

  // Cancel incoming
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }
}
