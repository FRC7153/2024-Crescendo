package com.frc7153.logging.datalogentry;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * For putting x, y points into log files that can be viewed with 
 * AdvantageScope
 */
public class PointsLogEntry {
    private String name;

    // Entries
    private DoubleArrayLogEntry xEntry;
    private DoubleArrayLogEntry yEntry;
 
    /**
     * Creates a new PointsLogEntry.
     * @param log
     * @param name
     * @param timestamp (0 to indicate now)
     */
    public PointsLogEntry(DataLog log, String name, long timestamp) {
        name = (name.endsWith("/") ? name : name + "/");
        this.name = name;

        xEntry = new DoubleArrayLogEntry(log, name + "x", timestamp);
        yEntry = new DoubleArrayLogEntry(log, name + "y", timestamp);
    }

    /**
     * Creates a new PointsLogEntry.
     * @param log
     * @param name
     */
    public PointsLogEntry(DataLog log, String name) {
        this(log, name, 0);
    }

    /**
     * Appends a new record to the log.
     * @param x list of x coordinates
     * @param y list of y coordinates
     * @param timestamp (0 to indicate now)
     */
    public void append(double[] x, double[] y, long timestamp) {
        xEntry.append(x, timestamp);
        yEntry.append(y, timestamp);
    }

    /**
     * Appends a new record to the log.
     * @param x list of x coordinates
     * @param y list of y coordinates
     */
    public void append(double[] x, double[] y) {
        append(x, y, 0);
    }

    /**
     * Appends a new record to the log. Each value in coordinates should alternate
     * between the X and the Y coordinate for each point, thus the length should
     * be even.
     * @param coordinates
     */
    public void append(double... coordinates) {
        if (coordinates.length % 2 != 0) {
            // Length is not even
            DriverStation.reportError(String.format(
                "Could not add points to '%s': number of coordinates must be even (is %d)",
                name,
                coordinates.length
            ), false);
            return;
        }

        // Create X, Y lists
        int total = coordinates.length/2;
        double[] x = new double[total];
        double[] y = new double[total];

        for (int i = 0; i < total; i++) {
            x[i] = coordinates[2 * i];
            y[i] = coordinates[2 * i + 1];
        }

        // Save data
        xEntry.append(x, 0);
        yEntry.append(y, 0);
    }
}
