package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

public class ShootCommand extends ConditionalCommand{

    public ShootCommand(Indexer indexer, boolean direction, boolean overrideSensor){
        super(
            new SequentialCommandGroup(
                new InstantCommand(() -> indexer.setIndexerVelocity(direction ? 400.0 : -400.0)),
                new WaitUntilCommand(() -> !indexer.detectingNote()),
                new WaitCommand(5.0),
                new InstantCommand(() -> StateController.setState(NoteState.EMPTY))
            ), 
            new PrintCommand("OPERATOR tried to SHOOT speaker without NOTE loaded!"), 
            () -> { return overrideSensor || StateController.getState().equals(NoteState.LOADED); }
        );

        addRequirements(indexer);

    }
    
}
