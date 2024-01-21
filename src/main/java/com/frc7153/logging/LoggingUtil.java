package com.frc7153.logging;

import java.util.Optional;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.WPILibVersion;

/**
 * Additional methods for logging data with AdvantageScope through WPILOG files
 */
public class LoggingUtil {
    /**
     * Adds metadata to a log file (see AdvantageScope's metadata tab). This
     * should be called ONCE per key (but will not error if called more than
     * once).
     * @param key
     * @param value
     * @param type Used for sorting
     */
    public static void addMetadata(String key, Object value, MetadataType type) {
        DataLog log = DataLogManager.getLog();

        int entry = log.start("RealMetadata/" + type.prefix + key, "string");
        log.appendString(entry, value.toString(), 0);
        log.finish(entry);
    }

    /**
     * Adds metadata to the default log file (see AdvantageScope's metadata tab)
     * @param key
     * @param value
     */
    public static void addMetadata(String key, Object value) {
        addMetadata(key, value, MetadataType.OTHER);
    }

    /**
     * Logs RoboRIO info. Then, waits for Driver Station to connect, then logs 
     * match info to metadata. This is non-blocking.
     */
    public static void addDefaultMetadata() {
        addMetadata("Startup TS", Timer.getFPGATimestamp(), MetadataType.TIMESTAMP_STARTUP);

        addMetadata("RoboRIO Comments", RobotController.getComments(), MetadataType.ROBORIO);
        addMetadata("RoboRIO FPGA Version", RobotController.getFPGAVersion(), MetadataType.ROBORIO);
        addMetadata("WPILib Version", WPILibVersion.Version, MetadataType.ROBORIO);

        (new Thread(LoggingUtil::waitForDSThread)).start();
    }

    // Wait for FMS attach (returns success or timeout)
    private static boolean waitForFMS() {
        double start = Timer.getFPGATimestamp();

        // timeout 5 mins
        try {
            while (Timer.getFPGATimestamp() < start + 300.0) {
                if (DriverStation.isFMSAttached()) return true;
                else Thread.sleep(1500); // 1.5 second wait
            }
        } catch (InterruptedException e) {
            DriverStation.reportWarning("Logging interrupted while waiting for FMS connection.", false);
        }

        return DriverStation.isFMSAttached();
    }

    // Runs in other thread to grab DS match info
    private static void waitForDSThread() {
        // Wait for Driver Station connection (no timeout)
        DriverStation.waitForDsConnection(0);
        addMetadata("DS Connection TS", Timer.getFPGATimestamp(), MetadataType.TIMESTAMP_NETWORK);
        
        // Wait for FMS connection (5 min timeout)
        if (waitForFMS()) addMetadata("FMS TS", Timer.getFPGATimestamp(), MetadataType.TIMESTAMP_NETWORK);
        else addMetadata("FMS TS", "Timed out (>5min)", MetadataType.TIMESTAMP_NETWORK);

        // Log FMS info
        addMetadata("Event Name", DriverStation.getEventName(), MetadataType.MATCH_INFO);
        addMetadata("Match Number", DriverStation.getMatchNumber(), MetadataType.MATCH_INFO);
        addMetadata("Match Type", DriverStation.getMatchType(), MetadataType.MATCH_INFO);
        addMetadata("Replay Number", DriverStation.getReplayNumber(), MetadataType.MATCH_INFO);
        addMetadata("Driver Station Location", DriverStation.getLocation().orElse(-1), MetadataType.MATCH_INFO);

        // This optional needs to be unwrapped
        Optional<Alliance> alliance = DriverStation.getAlliance();
        addMetadata(
            "Alliance", 
            alliance.isPresent() ? alliance.get().name() : "Unknown", 
            MetadataType.MATCH_INFO
        );
    }

    /**
     * Formats a generic Phoenix6 device (for logging)
     * @param device
     * @return Contains type, CAN id, and CAN bus name
     */
    public static String formatPhoenixDevice(ParentDevice device) {
        return String.format("CTRE %s #%d (bus '%s')",
            device.getClass().getSimpleName(), device.getDeviceID(), device.getNetwork()
        );
    }
}