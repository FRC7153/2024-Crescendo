package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.drive.SwerveBase;

public class TeleopDriveHeadingLockCommand extends Command {
  // Objects
  private SwerveBase base;
  private PIDController headingCorrectionPID = new PIDController(
    DriveConstants.kHEADING_CORRECTION_P,
    DriveConstants.kHEADING_CORRECTION_I,
    DriveConstants.kHEADING_CORRECTION_D
  );

  // Inputs
  private Supplier<Double> xSupply, ySupply;

  // Constructor
  public TeleopDriveHeadingLockCommand(
    SwerveBase base,
    Supplier<Double> ySupply, 
    Supplier<Double> xSupply
  ) {
    this.base = base;
    this.xSupply = xSupply;
    this.ySupply = ySupply;
  }

  // Execute
  @Override
  public void execute() {
    // Get position
    Pose2d pose = base.getPosition(false);

    // Drive
    base.driveFullFieldOriented(
      ySupply.get() * DriveConstants.kMAX_TELEOP_TRANSLATIONAL_SPEED,
      xSupply.get() * DriveConstants.kMAX_TELEOP_TRANSLATIONAL_SPEED,
      // Calculate angle
      headingCorrectionPID.calculate(
        pose.getRotation().getDegrees(),
        Units.radiansToDegrees(
          Math.tan( (pose.getX()) / (FieldConstants.kSPEAKER_POS.getY() - pose.getY()) )
        )
      )
    );
  }

  // End
  @Override
  public void end(boolean terminated) {
    base.driveFieldOriented(0.0, 0.0, 0.0);
  }

  // Interruption
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }
}
