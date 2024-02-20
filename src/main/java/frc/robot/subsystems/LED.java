package frc.robot.subsystems;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LEDConstants;

/**
 * Interfaces with the RaspberryPi controlling the LEDs
 */
public class LED implements Subsystem {
    private double value = -1.0;

    // NT entry
    // Values are published as integers to prevent rounding errors
    private IntegerPublisher ntPulsePublisher = 
        NetworkTableInstance.getDefault().getTable("LED").getIntegerTopic("pulse").publish();
    
    // Log
    private DoubleLogEntry logEntry = new DoubleLogEntry(DataLogManager.getLog(), "LED/pulse");

    public LED() {
        // Default to OFF
        ntPulsePublisher.setDefault((int)(LEDConstants.kOFF * 100));
        setPulse(LEDConstants.kOFF);
    }

    /**
     * Init default command (driver station color)
     */
    public void initDefaultCommand() {
        setDefaultCommand(new InstantCommand(
            this::setAllianceStationColor,
            this
        ));
    }

    /**
     * Sets the pulse width of the PWM signal.
     * @param pulse
     */
    public void setPulse(double pulse) {
        // Lazy set pulse
        if (value == pulse) return;
        value = pulse;

        ntPulsePublisher.set((int)(pulse*100));
        logEntry.append(pulse);
    }

    /**
     * Tells the Pi to use the DriverStation Alliance Color
     */
    public void setAllianceStationColor() {
        setPulse(2);
    }
}