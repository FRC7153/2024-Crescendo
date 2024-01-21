package com.frc7153.diagnostics.devices;

import java.util.ArrayList;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.frc7153.diagnostics.CheckableDevice;
import com.frc7153.logging.LoggingUtil;
import com.frc7153.math.MathUtils;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class TalonFXDevice extends CheckableDevice {
    private TalonFX talon;
    private ArrayList<StatusSignal<Boolean>> signals = new ArrayList<>(10);
    private StringBuilder faultMsg = new StringBuilder(); // cached on rawIsOK()
    private String id;

    // Logging
    private DoubleLogEntry motorTempLog;
    private DoubleLogEntry procTempLog;

    public TalonFXDevice(TalonFX talon) {
        this.talon = talon;
        id = LoggingUtil.formatPhoenixDevice(talon);
        
        signals.add(talon.getFault_BootDuringEnable());
        signals.add(talon.getFault_BridgeBrownout());
        signals.add(talon.getFault_DeviceTemp());
        signals.add(talon.getFault_FusedSensorOutOfSync());
        signals.add(talon.getFault_Hardware());
        signals.add(talon.getFault_OverSupplyV());
        signals.add(talon.getFault_ProcTemp());
        signals.add(talon.getFault_Undervoltage());
        signals.add(talon.getFault_UnlicensedFeatureInUse());
        signals.add(talon.getFault_UnstableSupplyV());

        motorTempLog = new DoubleLogEntry(DataLogManager.getLog(), String.format("Hardware/%s/Motor Temp", id), "F");
        procTempLog = new DoubleLogEntry(DataLogManager.getLog(), String.format("Hardware/%s/Proc Temp", id), "F");
    }

    @Override
    protected boolean isOk() {
        faultMsg.delete(0, faultMsg.length());
        boolean ok = true;

        // Check all faults
        for (StatusSignal<Boolean> ss : signals) {
            ss.refresh();

            if (ss.getValue()) {
                ok = false;
                faultMsg.append(ss.getName()).append(", ");
            }
        }

        return ok;
    }

    @Override
    public String getMessage() {
        return faultMsg.toString();
    }

    @Override
    public String getID() {
        return id;
    }
    
    @Override
    public void performLogging() {
        motorTempLog.append(MathUtils.celsiusToFahrenheit(talon.getDeviceTemp().getValue()));
        procTempLog.append(MathUtils.celsiusToFahrenheit(talon.getProcessorTemp().getValue()));
    }
}
