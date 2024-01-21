package com.frc7153.diagnostics.devices;

import com.frc7153.diagnostics.CheckableDevice;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class ADIS16470Device extends CheckableDevice {
    private ADIS16470_IMU imu;
    private String id;

    public ADIS16470Device(ADIS16470_IMU imu) {
        this.imu = imu;
        id = String.format("ADIS16470 IMU (%d)", imu.getPort());
    }

    @Override
    protected boolean isOk() {
        return imu.isConnected();
    }

    @Override
    public String getMessage() {
        return "The IMU is not connected!";
    }

    @Override
    public String getID() {
        return id;
    }

    @Override
    public void performLogging() {} // None needed
}
