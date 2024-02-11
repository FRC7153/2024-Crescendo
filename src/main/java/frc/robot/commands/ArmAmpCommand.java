package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class ArmAmpCommand extends RepeatCommand {
    /**
     * Arms the robot to shoot into the AMP.
     * Requires a NOTE to be LOADED.
     */
    public ArmAmpCommand(Shooter shooter) {
        super(new SequentialCommandGroup(
            // TODO set shooter, arm, LEDs
        ));
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
