package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

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
            new InstantCommand(() -> indexer.setIndexerVelocity(direction ? 400.0 : -400.0), indexer),
            // Wait until empty
            new WaitCommand(0.75), // TODO reverse shooting will require more time
            // Stop and set state
            new InstantCommand(indexer::stop, indexer),
            new InstantCommand(() -> StateController.setNoteState(NoteState.EMPTY))
        );
    }
}
