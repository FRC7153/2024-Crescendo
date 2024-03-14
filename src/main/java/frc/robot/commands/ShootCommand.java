package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class ShootCommand extends Command {
    private Indexer indexer;
    private boolean direction;

    /**
     * Runs the indexer to push a NOTE into the shooter wheels, or out the back.
     * Cancels incoming commands.
     * @param indexer
     * @param direction true = indexer runs forward, false = indexer reverse
     */
    public ShootCommand(Indexer indexer, boolean direction) {
        this.indexer = indexer;
        this.direction = direction;

        addRequirements(indexer);
    }

    // Init
    @Override
    public void initialize() {
        indexer.setIndexerVelocity(direction ? 700.0 : -700.0);
    }

    // End
    @Override
    public void end(boolean terminated) {
        indexer.stop();
    }

    // Interruption
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
