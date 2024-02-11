package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

/**
 * Sets the indexer speed to intake a NOTE, while running the shooter backwards slowly.
 * Sets the robot state to LOADED
 */
public class LoadShooterCommand extends SequentialCommandGroup {
    public LoadShooterCommand(Shooter shooter, Indexer indexer) {
        super(
            new InstantCommand(() -> shooter.setShootVelocity(-1.0), shooter),
            new InstantCommand(() -> indexer.setIndexerVelocity(400.0), indexer),
            new WaitUntilCommand(indexer::detectingNote),
            new InstantCommand(() -> indexer.setIndexerVelocity(0.0), indexer),
            new InstantCommand(() -> shooter.setShootVelocity(0.0), shooter),
            new InstantCommand(() -> StateController.setState(NoteState.LOADED))
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
