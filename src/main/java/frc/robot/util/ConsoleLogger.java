package frc.robot.util;

import java.io.PrintStream;
import java.io.IOException;
import java.io.OutputStream;

import org.ejml.simple.UnsupportedOperation;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Captures Java's Console messages to WPILogs
 */
public class ConsoleLogger {
    // Output
    private static StringLogEntry dataLogOut = new StringLogEntry(DataLogManager.getLog(), "System Out");

    // Original out
    private static PrintStream originalOut;

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
                dataLogOut.append(buffer.toString());
                buffer.delete(0, buffer.length());
            } else {
                // Append to buffer
                buffer.append((char)b);
            }

            originalOut.append((char)b);
        }

    }

    /**
     * Replaces System's out
     */
    public static void init() {
        originalOut = System.out;
        System.setOut(new PrintStream(new DataLogOutputStream(), true));
    }
}
