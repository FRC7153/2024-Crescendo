package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

/**
 * Arms the robot to shoot into the AMP.
 * Requires a NOTE to be LOADED or PROCESSING.
 */
public class ArmAmpCommand extends SequentialCommandGroup {
    public ArmAmpCommand(Shooter shooter, boolean overrideSensor) {
        super(
            // Wait until NOTE LOADED
            new WaitUntilCommand(() -> overrideSensor || StateController.getState().equals(NoteState.LOADED)),
            // Repeatedly set setpoints until interrupted
            new SequentialCommandGroup(
                // TODO set shooter, arm, LEDs
            ).repeatedly()
        );
    }

    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
