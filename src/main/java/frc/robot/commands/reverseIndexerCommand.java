package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Indexer;

public class ReverseIndexerCommand extends InstantCommand{
    /**
    * Reverses the indexer at the set velo
    */
    public ReverseIndexerCommand(Indexer indexer){
        super(() -> indexer.setIndexerVelocity(-500.0), indexer);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
