package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LED;
import frc.robot.Constants.LEDConstants;

/** Command that flashes the LED at a certain color 3 times */
public class FlashLEDCommand extends SequentialCommandGroup {
    public FlashLEDCommand(LED led, double pulse){
        super(
            singleFlashCommand(led, pulse),
            singleFlashCommand(led, pulse),
            singleFlashCommand(led, pulse)
        );

        addRequirements(led);
    }
    
    private static SequentialCommandGroup singleFlashCommand(LED led, double pulse) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> { led.setPulse(pulse); }),
            new WaitCommand(0.5),
            new InstantCommand(() -> { led.setPulse(LEDConstants.kOFF); }),
            new WaitCommand(0.5)
        );
    }
}