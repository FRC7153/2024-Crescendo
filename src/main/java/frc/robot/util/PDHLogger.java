package frc.robot.util;

import com.frc7153.diagnostics.CheckableDevice;
import com.frc7153.diagnostics.DiagUtil;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class PDHLogger extends CheckableDevice {
    private PowerDistribution hub;
    private String id;
    private StringBuilder message = new StringBuilder(); // Cached message

    // Log
    private DoubleLogEntry tempLog, currentLog, powerLog, voltLog;

    public PDHLogger(int CAN) {
        hub = new PowerDistribution(CAN, ModuleType.kRev);
        id = String.format("Rev PDH %d", hub.getModule());

        tempLog = new DoubleLogEntry(DataLogManager.getLog(), String.format("Hardware/%s/Temp", id), "C");
        currentLog = new DoubleLogEntry(DataLogManager.getLog(), String.format("Hardware/%s/Current", id), "Amps");
        powerLog = new DoubleLogEntry(DataLogManager.getLog(), String.format("Hardware/%s/Power", id), "Watts");
        voltLog = new DoubleLogEntry(DataLogManager.getLog(), String.format("Hardware/%s/Volt", id), "Volts");

        DiagUtil.addDevice(this);
    }

    @Override
    protected boolean isOk() {
        message.delete(0, message.length());
        boolean err = false;

        PowerDistributionFaults faults = hub.getFaults();
        
        // Check faults
        if (faults.Brownout) { message.append("Brownout, "); err = true; }
        if (faults.CanWarning) { message.append("CAN Warning, "); err = true; }
        if (faults.HardwareFault) { message.append("Hardware fault, "); err = true; }

        // Check current
        for (int i = 0; i <= 23; i++) {
            if (i == 22) continue; // Unused port

            if (hub.getCurrent(i) == 0) {
                message.append(String.format("No draw channel %d, ", i));
                err = true;
            }
        }

        return err;
    }

    @Override
    public String getMessage() {
        // Should be cached
        return message.toString();
    }

    @Override
    public void performLogging() {
        tempLog.append(hub.getTemperature());
        currentLog.append(hub.getTotalCurrent());
        powerLog.append(hub.getTotalPower());
        voltLog.append(hub.getVoltage());
    }

    @Override
    public String getID() {
        return id;
    }
}
