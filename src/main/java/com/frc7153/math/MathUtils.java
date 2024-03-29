package com.frc7153.math;

/**
 * Shared, static math utility class for operation used by a number of other packages here.
 */
public class MathUtils {
    /**
     * Wraps a double value from 0.0 to 1.0
     * @param value
     * @return
     */
    public static double wrap0To1(double value) { return value % 1.0; }

    /**
     * Takes an angle and normalizes it to a range of 0 - 360
     * @param angle
     * @return The angle, normalizes
     */
    public static double normalizeAngle360(double angle) {
        return angle - (360.0 * Math.floor(angle / 360.0));
    }

    /**
     * Takes an angle and normalizes it to a range of -180 to 180
     * @param angle
     * @return The angle, normalized
     */
    public static double normalizeAngle180(double angle) {
        angle = normalizeAngle360(angle);
        if (angle > 180) { angle -= 360.0; }
        return angle;
    }

    /**
     * Convert Celsius to Fahrenheit
     * @param celsius
     * @Return Fahrenheit
     */
    public static double celsiusToFahrenheit(double celsius) {
        return celsius * (9.0/5.0) + 32.0;
    }
}
