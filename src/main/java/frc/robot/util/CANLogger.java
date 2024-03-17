package frc.robot.util;

import com.ctre.phoenix6.jni.CANBusJNI;

import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.HardwareConstants;

/**
 * Singleton that logs the CAN (and CAN FD) status.
 */
public class CANLogger {
    // Preallocate objects
    private static CANStatus canStatus = new CANStatus();
    private static CANBusJNI canFDStatus = new CANBusJNI();

    // Log entries
    private static DoubleLogEntry busOffLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN/Bus Off Count");
    private static DoubleLogEntry percentUtilLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN/Percent Util", "%");
    private static DoubleLogEntry rxErr = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN/RX Error Count");
    private static DoubleLogEntry txErr = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN/TX Error Count");
    private static DoubleLogEntry txFull = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN/TX Buffer Full Count");

    private static DoubleLogEntry fdBusOffLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN FD/Bus Off Count");
    private static DoubleLogEntry fdPercentUtilLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN FD/Percent Util", "%");
    private static DoubleLogEntry fdRxErr = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN FD/RX Error Count");
    private static DoubleLogEntry fdTxErr = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN FD/TX Error Count");
    private static DoubleLogEntry fdTxFull = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN FD/TX Buffer Full Count");

    // Can't be constructed
    private CANLogger() {
        throw new UnsupportedOperationException("Utility class!");
    }

    /**
     * Appends data to the data log
     */
    public static void periodic() {
        // Log CAN
        CANJNI.getCANStatus(canStatus);
        
        busOffLog.append(canStatus.busOffCount);
        percentUtilLog.append(canStatus.percentBusUtilization);
        rxErr.append(canStatus.receiveErrorCount);
        txErr.append(canStatus.transmitErrorCount);
        txFull.append(canStatus.txFullCount);

        // Log CAN FD
        canFDStatus.JNI_GetStatus(HardwareConstants.kCANIVORE_BUS);

        fdBusOffLog.append(canFDStatus.busOffCount);
        fdPercentUtilLog.append(canFDStatus.busUtilization);
        fdRxErr.append(canFDStatus.rec);
        fdTxErr.append(canFDStatus.tec);
        fdTxFull.append(canFDStatus.txFullCount);
    }
}
