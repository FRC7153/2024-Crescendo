package com.frc7153.diagnostics.devices;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.frc7153.diagnostics.CheckableDevice;

public class CANCoderDevice extends CheckableDevice {
    private StatusSignal<Boolean> ss_badMagnet, ss_bde, ss_hardware, ss_uv, ss_unlicensed;

    private String id;

    public CANCoderDevice(CANcoder cancoder) {
        ss_badMagnet = cancoder.getFault_BadMagnet();
        ss_bde = cancoder.getFault_BootDuringEnable();
        ss_hardware = cancoder.getFault_Hardware();
        ss_uv = cancoder.getFault_Undervoltage();
        ss_unlicensed = cancoder.getFault_UnlicensedFeatureInUse();

        id = String.format("CTRE CANCoder %d (bus: %s)", cancoder.getDeviceID(), cancoder.getNetwork());
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
    
}
