package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.led.FlashLEDCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm.ArmState;

public class LoadShooterCommand extends SequentialCommandGroup {
    /**
     * Sets the indexer speed to intake a NOTE, without running the shooter.
     * Moves the arm.
     * Sets the robot state to LOADED.
     */
    public LoadShooterCommand(
        Arm arm, 
        Shooter shooter, 
        Intake intake,
        Indexer indexer, 
        LED led, 
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
            new InstantCommand(() -> { if (runIntake) intake.end(); }, intake),
            // Flash LEDs and restore positions
            new ParallelCommandGroup(
                new InstantCommand(() -> arm.setState(ArmPositions.kDEFAULT), arm)
                //new InstantCommand(flashLEDYellowCommand::schedule)
                // ^ TODO Flash LEDS
            )
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
