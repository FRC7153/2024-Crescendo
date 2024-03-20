package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.drive.SwerveBase;

public class ClimbBalancedCommand extends Command {
  // Subsystems
  private SwerveBase base;
  private Climber climber;

  // Speeds
  private double lSpeed, rSpeed;

  /**
   * Brings the robot up, adjusting the height of each climber to stabilize the roll.
   * Does not require the drive base.
   * Will reset the robot gyro's ROLL
   */
  public ClimbBalancedCommand(SwerveBase base, Climber climber) {
    this.base = base;
    this.climber = climber;

    addRequirements(climber);
  }

  // Init
  @Override
  public void initialize() {
    base.resetRoll();

    lSpeed = 0.8;
    rSpeed = 0.8;
  }

  // Execute
  @Override
  public void execute() {
    // Error
    double error = base.getRoll(); // should be 0

    lSpeed += (error * ClimberConstants.kCLIMB_ROLL_STABILIZE_P);
    rSpeed += (error * ClimberConstants.kCLIMB_ROLL_STABILIZE_P);

    // Set
    climber.setClimberVelocity(lSpeed, rSpeed);
  }

  // End
  @Override
  public boolean isFinished() {
    return (climber.isClimberFloored());
  }

  @Override
  public void end(boolean terminated) {
    climber.setClimberVelocity(0.0, 0.0);
  }

  // Cancel self
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
      return InterruptionBehavior.kCancelSelf;
  }
}
