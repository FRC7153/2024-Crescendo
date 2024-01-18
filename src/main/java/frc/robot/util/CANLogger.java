package frc.robot.util;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Singleton that logs the CAN status
 */
public class CANLogger {
    // Log entries
    private static DoubleLogEntry busOffLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN/Bus Off Count");
    private static DoubleLogEntry percentUtilLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN/Percent Util", "%");
    private static DoubleLogEntry rxErr = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN/Receive Error Count");
    private static DoubleLogEntry txErr = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN/Transmit Error Count");
    private static DoubleLogEntry txFull = 
        new DoubleLogEntry(DataLogManager.getLog(), "CAN/TX Buffer Full Count");

    // Can't be constructed
    private CANLogger() {
        throw new UnsupportedOperationException("Singleton class!");
    }

    /**
     * Appends data to the data log
     */
    public static void periodic() {
        CANStatus status = RobotController.getCANStatus();
        
        busOffLog.append(status.busOffCount);
        percentUtilLog.append(status.percentBusUtilization);
        rxErr.append(status.receiveErrorCount);
        txErr.append(status.transmitErrorCount);
        txFull.append(status.txFullCount);
    }
}
