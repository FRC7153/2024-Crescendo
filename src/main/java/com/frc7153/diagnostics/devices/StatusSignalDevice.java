package com.frc7153.diagnostics.devices;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.frc7153.diagnostics.CheckableDevice;
import com.frc7153.logging.LoggingUtil;

public class StatusSignalDevice extends CheckableDevice {
    private StatusSignal<?> signal;
    private String id;

    public StatusSignalDevice(StatusSignal<?> signal, ParentDevice device) {
        this.signal = signal;
        id = String.format("%s %s (%s)", 
            LoggingUtil.formatPhoenixDevice(device), signal.getName(), signal.getUnits()
        );
    }

    @Override
    protected boolean isOk() {
        return signal.getStatus().isOK();
    }

    @Override
    public String getMessage() {
        StatusCode status = signal.getStatus();
        return status.getName() + ": " + status.getDescription();
    }

    @Override
    public String getID() {
        return id;
    }
    
}
