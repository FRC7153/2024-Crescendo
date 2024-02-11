package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Runs the intake in a specified direction (forward or backward) until cancelled.
 */
public class IntakeCommand extends Command {
    private Intake intake;
    private boolean direction;

    /**
     * @param intake
     * @param direction true = in, false = out
     */
    public IntakeCommand(Intake intake, boolean direction) {
        this.intake = intake;
        this.direction = direction;

        addRequirements(intake);
    }

    // Start
    @Override
    public void initialize() {
        if (direction) intake.enableIntake();
        else intake.reverseIntake();
    }

    // End
    @Override
    public void end(boolean terminated) {
        intake.end();
    }

    // Never finishes
    @Override
    public boolean isFinished() { return false; }

    // Cancels itself
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
