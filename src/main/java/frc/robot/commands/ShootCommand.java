package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;

public class ShootCommand extends SequentialCommandGroup {
    /**
     * Runs the indexer to push a NOTE into the shooter wheels, or out the back.
     * Requires a NOTE to be LOADED.
     * @param indexer
     * @param direction true = indexer runs forward, false = indexer reverse
     */
    public ShootCommand(Indexer indexer, boolean direction){
        super(
            // Run intake
            new InstantCommand(() -> indexer.setIndexerVelocity(direction ? 700.0 : -700.0), indexer),
            // Wait until empty
            new WaitCommand(100), // TODO reverse shooting will require more time, was 0.75
            // Stop
            new InstantCommand(indexer::stop, indexer)
        );
    }
}
