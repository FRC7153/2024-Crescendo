package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.led.FlashLEDCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LED;

public class reverseIndexerCommand extends SequentialCommandGroup{
    
    public reverseIndexerCommand(Indexer indexer, LED led){
    /*
     * Reverses the indexer at the set velo
     */
        super(
        new InstantCommand(() -> indexer.setIndexerVelocity(-500.0), indexer),
        new FlashLEDCommand(led, LEDConstants.kRED)
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
