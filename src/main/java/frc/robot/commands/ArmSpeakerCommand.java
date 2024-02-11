package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class ArmSpeakerCommand extends RepeatCommand {
    /**
     * Arms the robot to shoot into the SPEAKER.
     * Requires a NOTE to be LOADED or PROCESSING.
     */
    public ArmSpeakerCommand(Shooter shooter) {
        super(new SequentialCommandGroup(
            new InstantCommand(() -> shooter.setShootVelocity(40), shooter) // Set shoot velocity
            // TODO set arm (with requirement)
            // TODO check setpoints, set LEDs (with requirement)
        ));
    }

    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
