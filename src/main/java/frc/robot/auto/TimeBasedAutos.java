package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveBase;

public class TimeBasedAutos extends SequentialCommandGroup {
  /**
   * Time based auto. May shoot note in speaker, then drives forward.
   * Does not reset odometry - robot can be placed anywhere.
   * @param base
   * @param arm
   * @param shooter
   * @param indexer
   * @param shootPreloaded if a note is preloaded and should be shot into the speaker
   */
  public TimeBasedAutos(
    SwerveBase base, 
    Arm arm, 
    Shooter shooter, 
    Indexer indexer, 
    boolean shootPreloaded
  ) {
    super(
      new ConditionalCommand(
        // Shoot preloaded note
        new SequentialCommandGroup(
          new InstantCommand(indexer::stop, indexer),
          new InstantCommand(() -> arm.setState(ArmPositions.kSUBWOOFER_SPEAKER_REAR), arm),
          new InstantCommand(() -> shooter.setShootVelocity(58.0), shooter),
          new WaitCommand(2.0),
          new InstantCommand(() -> indexer.setIndexerVelocity(700.0), indexer),
          new WaitCommand(2.15),
          new InstantCommand(indexer::stop, indexer)
        ),
        // Don't shoot preloaded note
        new PrintCommand("Not shooting preloaded note in auto"), 
        () -> shootPreloaded
      ),
      // Drive forward
      new InstantCommand(() -> base.driveFieldOriented(7.0, 0.0, 0.0), base),
      new WaitCommand(1.8),
      new InstantCommand(() -> base.driveFieldOriented(0.0, 0.0, 0.0), base)
    );

    addRequirements(base, arm, shooter, indexer);
    }
}
