package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

public class ShootCommand extends SequentialCommandGroup {
    /**
     * Runs the indexer to push a NOTE into the shooter wheels, or out the back.
     * Requires a NOTE to be LOADED.
     * @param indexer
     * @param direction true = indexer runs forward, false = indexer reverse
     * @param overrideSensor
     */
    public ShootCommand(Indexer indexer, boolean direction, boolean overrideSensor){
        super(
            new InstantCommand(() -> indexer.setIndexerVelocity(direction ? 400.0 : -400.0), indexer),
            new WaitUntilCommand(() -> !indexer.detectingNote()),
            new WaitCommand(5.0),
            new InstantCommand(() -> StateController.setState(NoteState.EMPTY))
        );
    }
}
