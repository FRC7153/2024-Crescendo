package com.frc7153.diagnostics.devices;

import com.frc7153.diagnostics.CheckableDevice;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

public class NTEntryDevice extends CheckableDevice {
    private NetworkTableEntry entry;
    private double timeout;

    // Log
    private DoubleLogEntry ageLog;

    // Constructors
    public NTEntryDevice(NetworkTableEntry entry, double timeout) {
        this.entry = entry;
        this.timeout = timeout;

        ageLog = new DoubleLogEntry(
            DataLogManager.getLog(),
            String.format("/Hardware/NT/%s/age", entry.getName()),
            "s");
    }

    public NTEntryDevice(NetworkTableEntry entry) {
        this(entry, 3.0);
    }

    @Override
    protected boolean isOk() {
        return entry.exists() && (Timer.getFPGATimestamp() - entry.getLastChange() > timeout);
    }

    @Override
    public String getMessage() {
        if (!entry.exists()) return "Entry does not exist!";
        else return String.format("Last change %fs ago! (%fs is acceptable)",
            Timer.getFPGATimestamp() - entry.getLastChange(),
            timeout
        );
    }

    @Override
    public void performLogging() {
        ageLog.append(Timer.getFPGATimestamp() - entry.getLastChange());
    }

    @Override
    public String getID() {
        return String.format("NT - %s", entry.getName());
    }
}
