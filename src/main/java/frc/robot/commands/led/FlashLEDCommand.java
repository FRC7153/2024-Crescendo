package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LED;
import frc.robot.Constants;

public class FlashLEDCommand extends Command{
    private LED led;

    public FlashLEDCommand(double pulseArgument){
        super(
            new SequentialCommandGroup(
                new onCommand(() -> LED.setPulse(pulseArgument)),

                new waitCommand(() -> LED.setPulse(kOFF)),

                new offCommand(() -> LED.setPulse(kOFF))
            )
            );

        addRequirements(LED);
    }
    
}