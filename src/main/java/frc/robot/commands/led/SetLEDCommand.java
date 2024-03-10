package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;

public class SetLEDCommand extends InstantCommand {
  /**
   * Sets the LEDs once
   * @param led
   * @param pulse
   */
  public SetLEDCommand(LED led, double pulse) {
    super(() -> { led.setPulse(pulse); }, led);
  }
}
