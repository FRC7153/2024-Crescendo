package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.led.FlashLEDCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

public class LoadShooterCommand extends SequentialCommandGroup {
    /**
     * Sets the indexer speed to intake a NOTE, while running the shooter backwards slowly.
     * Does NOT move the arm.
     * Sets the robot state to LOADED.
     */
    public LoadShooterCommand(Shooter shooter, Indexer indexer, LED led) {
        super(
            // Start motors
            new InstantCommand(() -> shooter.setShootVelocity(0.0), shooter),
            new InstantCommand(() -> indexer.setIndexerVelocity(160.0), indexer),
            // Wait for piece
            new WaitCommand(0.75),
            new WaitUntilCommand(indexer::detectingNote),
            // Stop motors
            new InstantCommand(indexer::stop, indexer),
            // Flash LEDs, if not done already
            new ConditionalCommand(
                // LEDs flash already
                new InstantCommand(() -> StateController.setNoteState(NoteState.LOADED)), 
                // LEDs haven't flashed
                new ParallelCommandGroup(
                    new FlashLEDCommand(led, LEDConstants.kYELLOW),
                    new InstantCommand(() -> StateController.setNoteState(NoteState.LOADED))
                ), 
                () -> StateController.getNoteState().equals(NoteState.PROCESSING)
            )
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
