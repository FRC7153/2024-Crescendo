package com.frc7153.diagnostics.devices;

import com.frc7153.diagnostics.CheckableDevice;
import com.revrobotics.CANSparkMax;
import static com.revrobotics.CANSparkMax.FaultID;

public class CANSparkMaxDevice extends CheckableDevice {
    // Faults to check
    private static FaultID[] faults = {
        FaultID.kBrownout, FaultID.kCANRX, FaultID.kCANTX, FaultID.kDRVFault,
        FaultID.kEEPROMCRC, FaultID.kHasReset, FaultID.kIWDTReset, FaultID.kMotorFault,
        FaultID.kSensorFault, FaultID.kOtherFault
    };

    private CANSparkMax spark;
    private String id;
    private StringBuilder faultMsg = new StringBuilder(); // this is cached when rawIsOK() is called

    public CANSparkMaxDevice(CANSparkMax spark) {
        this.spark = spark;
        id = "CAN Spark Max ID " + spark.getDeviceId();
    }

    @Override
    protected boolean isOk() {
        faultMsg.delete(0, faultMsg.length());
        boolean ok = true;
        
        // Check all faults
        for (FaultID f : faults) {
            if (spark.getFault(f)) {
                faultMsg.append(f.toString()).append(", ");
                ok = false;
            }
        }

        return ok;
    }

    @Override
    public String getMessage() {
        // Relies on rawIsOK() being called already
        return faultMsg.toString();
    }

    @Override
    public String getID() {
        return id;
    }
    
}
