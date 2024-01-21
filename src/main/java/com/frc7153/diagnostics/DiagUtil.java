package com.frc7153.diagnostics;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.frc7153.diagnostics.devices.ADIS16470Device;
import com.frc7153.diagnostics.devices.CANCoderDevice;
import com.frc7153.diagnostics.devices.CANSparkMaxDevice;
import com.frc7153.diagnostics.devices.PhotonCameraDevice;
import com.frc7153.diagnostics.devices.TalonFXDevice;
import com.frc7153.logging.LoggingUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Diagnostics utility functions
 */
public class DiagUtil {
    // DEVICES THAT CAN BE CHECKED

    /**
     * Generic device
     * @param device
     */
    public static void addDevice(CheckableDevice device) { Diagnostics.getInstance().addDevice(device); }

    /**
     * Adds a new CANCoder device to be checked
     * @param cancoder
     */
    public static void addDevice(CANcoder cancoder) { Diagnostics.getInstance().addDevice(new CANCoderDevice(cancoder)); }

    /**
     * Adds a new CAN Spark Max device to be checked
     * @param spark
     */
    public static void addDevice(CANSparkMax spark) { Diagnostics.getInstance().addDevice(new CANSparkMaxDevice(spark)); }

    /**
     * Adds a new TalonFX device to be checked
     * @param talon
     */
    public static void addDevice(TalonFX talon) { Diagnostics.getInstance().addDevice(new TalonFXDevice(talon)); }

    /**
     * Adds a new ADIS16470 to be checked
     * @param imu
     */
    public static void addDevice(ADIS16470_IMU imu) { Diagnostics.getInstance().addDevice(new ADIS16470Device(imu)); }

    /**
     * Adds a new PhotonCamera to be checked
     * @param camera
     */
    public static void addDevice(PhotonCamera camera) { Diagnostics.getInstance().addDevice(new PhotonCameraDevice(camera)); }

    // EVALUATIONS

    /**
     * Checks whether a StatusCode is good
     * @param code
     * @param device Device that caused this StatusCode
     */
    public static void evaluateResponse(StatusCode code, ParentDevice device) {
        if (!code.isOK()) {
            Diagnostics.getInstance().appendFailedResponse((String.format("FAILED response (%s): %s: %s (%s)",
                (code.isError() ? "error" : "warning"), LoggingUtil.formatPhoenixDevice(device), code.getName(), code.getDescription()
            )));
        }
    }

    /**
     * Checks whether a REVLibError is good
     * @param error
     * @param device the CANSparkMax that caused this error
     */
    public static void evaluateResponse(REVLibError error, CANSparkMax device) {
        if (error != REVLibError.kOk) {
            Diagnostics.getInstance().appendFailedResponse((String.format("FAILED response: CANSparkMax %d: %s",
                device.getDeviceId(), error.name()
            )));
        }
    }

    /**
     * Adds an error to the diagnostics log. For example, if something doesn't boot or load correctly.
     * @param msg
     * @param args (will be formatted with these args)
     */
    public static void criticalError(String msg, Object... args) {
        String fMsg = String.format(msg, args);
        
        DriverStation.reportWarning(fMsg, false);
        Diagnostics.getInstance().appendFailedResponse(fMsg);
    }
}
