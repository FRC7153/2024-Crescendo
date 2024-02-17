package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.led.DriverStationLEDCommand;

/**
 * Interfaces with the RaspberryPi controlling the LEDs
 */
public class LED implements Subsystem {
    private double value = -1.0;

    // NT entry
    private DoublePublisher ntPublisher = 
        NetworkTableInstance.getDefault().getTable("LED").getDoubleTopic("pulse").publish();
    
    // Log
    private DoubleLogEntry logEntry = new DoubleLogEntry(DataLogManager.getLog(), "LED/pulse");

    public LED() {
        // Default to OFF
        ntPublisher.setDefault(LEDConstants.kOFF);
        setPulse(LEDConstants.kOFF);
    }

    /**
     * Init default command (driver station color)
     */
    public void initDefaultCommand() {
        setDefaultCommand(new DriverStationLEDCommand(this));
    }

    /**
     * Sets the pulse width of the PWM signal.
     * @param pulse
     */
    public void setPulse(double pulse) {
        // Lazy set pulse
        if (value == pulse) return;
        value = pulse;

        ntPublisher.set(pulse);
        logEntry.append(pulse);
    }
}