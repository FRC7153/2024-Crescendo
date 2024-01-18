package com.frc7153.diagnostics;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

/**
 * Singleton class for managing diagnostics
 */
public class Diagnostics {
    // Singleton object
    private static Diagnostics singleton;

    /**
     * @return The singleton instance of Diagnostics
     */
    public static Diagnostics getInstance() {
        if (singleton == null) singleton = new Diagnostics();
        return singleton;
    }

    // NT instances
    private BooleanPublisher nt_ok;
    private StringPublisher nt_msg;

    // Logger instances
    private StringLogEntry logEntry;

    // Devices to check
    private ArrayList<CheckableDevice> devices;
    private boolean errorThrown = false; // If appendFailedResponse() is called

    // Timer
    private Timer timer;

    /**
     * Private constructor, creates a new Diagnostics object.
     * This is a singleton
     */
    private Diagnostics() {
        // Init NT values
        NetworkTable nt = NetworkTableInstance.getDefault().getTable("Diagnostics");
        nt_ok = nt.getBooleanTopic("Diagnostics OK").publish();
        nt_msg = nt.getStringTopic("Diagnostics Message").publish();

        nt_ok.set(true);
        nt_msg.set("OK");

        nt_ok.setDefault(false);
        nt_msg.setDefault("NT using default values");

        // Init log
        logEntry = new StringLogEntry(DataLogManager.getLog(), "Diagnostics");

        // Init lists
        devices = new ArrayList<CheckableDevice>();

        // Init timer
        timer = new Timer();
        // Runs every second
        timer.scheduleAtFixedRate(new PeriodicCheck(), 0, 1000);
    }

    /** This will run periodically */
    private class PeriodicCheck extends TimerTask {
        private boolean ok;
        private StringBuilder msg = new StringBuilder();

        @Override
        public void run() {
            // Init
            ok = !errorThrown;
            msg.delete(0, msg.length());

            if (errorThrown) { msg.append("Evaluated bad response, check logs.\n"); }

            // Check each device
            for (CheckableDevice d : devices) {
                boolean d_ok = d.getOK();
                String d_id = d.getID();
                String d_msg = (d_ok ? "OK" : d.getMessage());

                if (!d_ok) { // Device is not ok
                    ok = false;

                    msg.append(d_id)
                        .append(": ")
                        .append(d_msg)
                        .append("; \n");
                }

                if (d.hasChanged()) { // Device OK has changed
                    String warning = String.format("%s: %s (%s)",
                        (d_ok ? "OK" : "NOT OK"), d_id, d_msg
                    );
                    
                    DriverStation.reportWarning("Device update: " + warning, false);
                    logEntry.append(warning);
                }
            }

            // Send to NT
            nt_ok.set(ok);

            if (ok) nt_msg.set("OK (checked " + devices.size() + " devices)");
            else nt_msg.set(msg.toString());
        }
    }

    /**
     * Adds a device that will be periodically checked.
     * @param device
     */
    public void addDevice(CheckableDevice device) { devices.add(device); }

    /**
     * Logs a failed response (ie, from a motor controller) to the log and sets the
     * error thrown flag
     * @param msg
     */
    public void appendFailedResponse(String msg) {
        logEntry.append(msg);
        errorThrown = true;
    }
}
