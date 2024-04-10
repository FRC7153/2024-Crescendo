package com.frc7153.diagnostics.devices;

import org.photonvision.PhotonCamera;

import com.frc7153.diagnostics.CheckableDevice;

import edu.wpi.first.wpilibj.Timer;

public class PhotonCameraDevice extends CheckableDevice {
    private PhotonCamera camera;
    private String id;
    private double timeSinceLastTarget = 0.0;

    public PhotonCameraDevice(PhotonCamera camera) {
        this.camera = camera;

        id = String.format("PhotonCamera '%s'", camera.getName());
    }

    @Override
    protected boolean isOk() {
        // Note: in Driver Mode, the camera does not need to be producing targets
        try {
            timeSinceLastTarget = camera.getLatestResult().getTimestampSeconds() - Timer.getFPGATimestamp();
            return camera.isConnected() && (camera.getDriverMode() || timeSinceLastTarget < 10.0);
        } catch (Exception e) {
            return false;
        }
    }

    @Override
    public String getMessage() {
        if (!camera.isConnected()) return "Not connected";
        if (timeSinceLastTarget >= 10.0) return "Last target too stale";
        return "OK";
    }

    @Override
    public String getID() {
        return id;
    }

    @Override
    public void performLogging() {} // none needed
}
