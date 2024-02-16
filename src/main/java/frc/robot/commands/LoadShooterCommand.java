package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

public class LoadShooterCommand extends SequentialCommandGroup {
    /**
     * Sets the indexer speed to intake a NOTE, while running the shooter backwards slowly.
     * Does NOT move the arm.
     * Sets the robot state to LOADED.
     */
    public LoadShooterCommand(Shooter shooter, Indexer indexer) {
        super(
            new InstantCommand(() -> shooter.setShootVelocity(0.0), shooter),
            new InstantCommand(() -> indexer.setIndexerVelocity(160.0), indexer),
            //new WaitUntilCommand(indexer::detectingNote),
            new WaitCommand(0.75),
            new WaitUntilCommand(() -> indexer.getMotorCurrent() >= 20),
            //new InstantCommand(() -> indexer.setIndexerVelocity(0.0), indexer),
            //new InstantCommand(() -> shooter.setShootVelocity(0.0), shooter),
            new InstantCommand(() -> indexer.setIndexerPos(13.0, true), indexer),
            new WaitCommand(0.3),
            new WaitUntilCommand(() -> Math.abs(indexer.getIndexerPos() - 13.0) < 13.0),
            new InstantCommand(indexer::stop, indexer),
            new InstantCommand(() -> StateController.setState(NoteState.LOADED))
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
