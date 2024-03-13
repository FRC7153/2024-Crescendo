package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;

public class IndexerRegripCommand extends SequentialCommandGroup {
  /**
   * Quickly runs indexer wheels backward, to regrip NOTES after being intake'd
   * @param indexer
   */
  public IndexerRegripCommand(Indexer indexer) {
    super(
      new InstantCommand(() -> indexer.setIndexerVelocity(-450.0), indexer),
      new WaitCommand(0.4),
      new InstantCommand(indexer::stop, indexer)
    );
  }

  @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
