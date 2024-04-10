package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class ShootCommand extends Command {
    private Indexer indexer;
    private boolean direction;

    private Supplier<Boolean> fastModeSupplier;

    /**
     * Runs the indexer to push a NOTE into the shooter wheels, or out the back.
     * Cancels incoming commands.
     * @param indexer
     * @param direction true = indexer runs forward, false = indexer reverse
     * @param fastMode true = shoot quickly
     */
    public ShootCommand(Indexer indexer, boolean direction, Supplier<Boolean> fastMode) {
        this.indexer = indexer;
        this.direction = direction;
        this.fastModeSupplier = fastMode;

        addRequirements(indexer);
    }

    // Init
    @Override
    public void initialize() {
        double speed = fastModeSupplier.get() ? 1500.0 : 730.0;
        indexer.setIndexerVelocity(direction ? speed : -speed);
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
