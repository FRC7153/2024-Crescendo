package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.HardwareConstants;

/**
 * Singleton that logs the CAN (and CAN FD) status
 */
public class CANLogger {
    /** CTRE says the method to get the CAN FD bus status is resource-intensive, so we'll only log
     * it once a second */
    private static double lastFDLog = 0.0;

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
        CANStatus status = RobotController.getCANStatus();
        
        busOffLog.append(status.busOffCount);
        percentUtilLog.append(status.percentBusUtilization);
        rxErr.append(status.receiveErrorCount);
        txErr.append(status.transmitErrorCount);
        txFull.append(status.txFullCount);

        // Log CAN FD
        if (Timer.getFPGATimestamp() - lastFDLog >= 1.0) {
            CANBusStatus fdStatus = CANBus.getStatus(HardwareConstants.kCANIVORE_BUS);

            fdBusOffLog.append(fdStatus.BusOffCount);
            fdPercentUtilLog.append(fdStatus.BusUtilization);
            fdRxErr.append(fdStatus.REC);
            fdTxErr.append(fdStatus.TEC);
            fdTxFull.append(fdStatus.TxFullCount);

            lastFDLog = Timer.getFPGATimestamp();
        }
    }
}
