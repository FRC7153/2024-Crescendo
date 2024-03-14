package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;

public class LoadShooterCommand extends SequentialCommandGroup {
    /**
     * Sets the indexer speed to intake a NOTE, without running the shooter.
     * Moves the arm.
     * Expects the Regrip command to be run directly afterwards
     */
    public LoadShooterCommand(
        Arm arm, 
        Shooter shooter, 
        Intake intake,
        Indexer indexer, 
        ArmState armLoadPos, 
        boolean runIntake
    ) {
        super(
            // Move arm
            new InstantCommand(() -> arm.setState(armLoadPos), arm),
            // Start motors
            new InstantCommand(() -> shooter.setShootVelocity(0.0), shooter),
            new InstantCommand(() -> indexer.setIndexerVelocity(850.0), indexer),
            new InstantCommand(() -> { if (runIntake) intake.enableIntake(); }, intake),
            // Wait for piece
            new WaitCommand(0.75),
            new InstantCommand(indexer::flushColorSensorQueue),
            new WaitUntilCommand(indexer::getHasDetectedNote),
            // Stop motors
            new InstantCommand(intake::end, intake),
            new InstantCommand(() -> { if (runIntake) intake.end(); }, intake)
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
