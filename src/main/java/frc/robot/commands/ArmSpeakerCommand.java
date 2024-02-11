package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

/**
 * Arms the robot to shoot into the SPEAKER.
 * Requires a NOTE to be LOADED or PROCESSING.
 */
public class ArmSpeakerCommand extends SequentialCommandGroup {

    public ArmSpeakerCommand(Shooter shooter, boolean overrideSensor) {
        super(
            // Wait until NOTE LOADED
            new WaitUntilCommand(() -> overrideSensor || StateController.getState().equals(NoteState.LOADED)),
            // Repeatedly set setpoints and output if ready until interrupted
            new SequentialCommandGroup(
                new InstantCommand(() -> shooter.setShootVelocity(40), shooter) // Set shoot velocity
                // TODO set arm (with requirement)
                // TODO check setpoints, set LEDs (with requirement)
            ).repeatedly()
        );
    }

    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
