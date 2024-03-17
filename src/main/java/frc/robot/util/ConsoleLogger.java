package frc.robot.util;

import java.io.PrintStream;
import java.util.ArrayList;
import java.io.IOException;
import java.io.OutputStream;

import org.ejml.simple.UnsupportedOperation;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

/**
 * Captures Java's Console messages to WPILogs
 */
public class ConsoleLogger {
    /** Message cached (before system startup) */
    private static class CachedMessage {
        private final String message;
        private final double systemTime;

        private CachedMessage(String message, double systemTime) {
            this.message = message;
            this.systemTime = systemTime;
        }
    }

    // Output
    private static StringLogEntry dataLogOut = null;
    private static ArrayList<CachedMessage> bootUpCache = new ArrayList<>();

    // Original out
    private static PrintStream originalOut = null;

    /** Can't be constructed */
    private ConsoleLogger() {
        throw new UnsupportedOperation("This is a utility class!");
    }

    // Output stream to data log
    private static class DataLogOutputStream extends OutputStream {
        private StringBuilder buffer = new StringBuilder();

        @Override
        public void write(int b) throws IOException {
            if (b == '\n' || buffer.length() > 800) {
                // Flush buffer
                if (dataLogOut == null) { // Robot program not started yet
                    bootUpCache.add(new CachedMessage(buffer.toString(), System.currentTimeMillis()));
                } else { // Robot program started
                    dataLogOut.append(buffer.toString());
                }

                buffer.delete(0, buffer.length());
            } else {
                // Append to buffer
                buffer.append((char)b);
            }

            originalOut.append((char)b);
        }
    }

    /**
     * Replaces System's out.
     * Will only cache messages until {@code robotProgramRunning()} is called
     */
    public static void init() {
        if (originalOut != null) {
            System.out.println("ConsoleLogger.init() called multiple times!");
            return;
        }

        originalOut = System.out;
        System.setOut(new PrintStream(new DataLogOutputStream(), true));
    }

    /**
     * Empties cache to WPILog file, then starts logging to that WPILog file only
     */
    public static void robotProgramRunning() {
        if (dataLogOut != null) {
            System.out.println("ConsoleLogger.robotProgramRunning() called multiple times!");
            return;
        }

        // Create log and get System time/FGPA time offset
        dataLogOut = new StringLogEntry(DataLogManager.getLog(), "System Out");
        double timeOffset = Timer.getFPGATimestamp() - System.currentTimeMillis();

        // Log cached messages
        for (CachedMessage msg : bootUpCache) {
            dataLogOut.append(msg.message, (long)(msg.systemTime + timeOffset));
        }

        // Log
        System.out.printf("System out now logging to WPILOG, %d messages cached\n", bootUpCache.size());

        // Dump cache
        bootUpCache.clear();
        bootUpCache = null;
    }
}
