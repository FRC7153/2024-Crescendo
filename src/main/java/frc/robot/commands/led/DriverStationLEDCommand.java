package frc.robot.commands.led;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LED;

public class DriverStationLEDCommand extends Command {
    // LED
    private LED led;

    /**
     * Sets the LEDs to the DriverStation alliance color
     */
    public DriverStationLEDCommand(LED led) {
        this.led = led;
        addRequirements(led);
    }

    // Set the LED colors
    @Override
    public void initialize() { execute(); }

    @Override
    public void execute() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            led.setPulse(
                alliance.get().equals(Alliance.Blue) ? LEDConstants.kBLUE : LEDConstants.kRED
            );
        } else {
            led.setPulse(LEDConstants.kOFF);
        }
    }

    @Override
    public void end(boolean terminated) {
        led.setPulse(LEDConstants.kOFF);
    }

    // Cancel this command when a new one is called
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
