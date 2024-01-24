package com.frc7153.diagnostics.devices;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.frc7153.diagnostics.CheckableDevice;
import com.frc7153.logging.LoggingUtil;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class CANCoderDevice extends CheckableDevice {
    private CANcoder cancoder;
    private StatusSignal<Boolean> ss_badMagnet, ss_bde, ss_hardware, ss_uv, ss_unlicensed;

    private String id;

    // Logging
    private StringLogEntry magnetHealthLog;

    public CANCoderDevice(CANcoder cancoder) {
        this.cancoder = cancoder;

        ss_badMagnet = cancoder.getFault_BadMagnet();
        ss_bde = cancoder.getFault_BootDuringEnable();
        ss_hardware = cancoder.getFault_Hardware();
        ss_uv = cancoder.getFault_Undervoltage();
        ss_unlicensed = cancoder.getFault_UnlicensedFeatureInUse();

        id = LoggingUtil.formatPhoenixDevice(cancoder);

        magnetHealthLog = new StringLogEntry(DataLogManager.getLog(), String.format("Hardware/%s/Magnet Health", id));
    }

    @Override
    protected boolean isOk() {
        BaseStatusSignal.refreshAll(ss_badMagnet, ss_bde, ss_hardware, ss_uv, ss_unlicensed);

        return !(ss_badMagnet.getValue() ||
            ss_bde.getValue() ||
            ss_hardware.getValue() ||
            ss_uv.getValue() ||
            ss_unlicensed.getValue());
    }

    @Override
    public String getMessage() {
        // isOk() is called first, so signals don't need to refresh again
        
        return (ss_badMagnet.getValue() ? "bad magnet, " : "") +
            (ss_bde.getValue() ? "boot during enable fault, " : "") +
            (ss_hardware.getValue() ? "hardware fault, " : "") +
            (ss_uv.getValue() ? "undervoltage, " : "") +
            (ss_unlicensed.getValue() ? "license error" : "");
    }

    @Override
    public String getID() {
        return id;
    }
    
    @Override
    public void performLogging() {
        magnetHealthLog.append(cancoder.getMagnetHealth().getName());
    }
}
