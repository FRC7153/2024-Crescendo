package frc.robot.subsystems;

import com.frc7153.diagnostics.DiagUtil;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedBoolean;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShooterConstants;

public class Indexer implements Subsystem {
    //Devices
    private CANSparkMax indexer = new CANSparkMax(HardwareConstants.kINDEXER_CAN, MotorType.kBrushless);
    private RelativeEncoder indexerEncoder = indexer.getEncoder();

    // Sensors
    private BooleanSubscriber colorSensorTarget;

    // Control
    private SparkPIDController indexerControl;

    //Logging
    private DoubleLogEntry indexerSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Indexer/Setpoint Velocity", "rps");
    private DoubleLogEntry indexerVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Indexer/Velocity", "rps");

    public Indexer(){
        // Init motors
        indexer.setIdleMode(IdleMode.kBrake);
        indexer.setInverted(true);
        indexer.setSmartCurrentLimit(ShooterConstants.kINDEXER_CURRENT_LIMIT);

        indexerControl = indexer.getPIDController();
        indexerControl.setP(ShooterConstants.kINDEXER_VELO_P, 0);
        indexerControl.setI(ShooterConstants.kINDEXER_VELO_I, 0);
        indexerControl.setD(ShooterConstants.kINDEXER_VELO_D, 0);

        indexer.burnFlash();

        // Init sensors
        NetworkTable sensorTable = NetworkTableInstance.getDefault().getTable("SecondaryPiSensors");
        colorSensorTarget = sensorTable.getBooleanTopic("Target").subscribe(false);

        DiagUtil.addDevice(indexer);
        DiagUtil.addDevice(colorSensorTarget.getTopic());

        indexerSetpointLog.append(0.0);

        register();
    }
    /**
     * Sets velocity in rpm
     * @param velocity
     */
    public void setIndexerVelocity(double velocity) {
        indexerControl.setReference(velocity / ShooterConstants.kINDEXER_RATIO, ControlType.kVelocity, 0);
        indexerSetpointLog.append(velocity);
    }

    /**
     * Stops the motor
     */
    public void stop() {
        indexer.disable();
        indexerSetpointLog.append(0.0);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new InstantCommand(this::stop, this));
    }

    /**
     * Flushes the color sensor's NetworkTables queue
     */
    public void flushColorSensorQueue() {
        colorSensorTarget.readQueueValues();
    }

    /**
     * @return If the color sensor has reported seeing a NOTE since the last call or the last call to
     * to {@code flushColorSensorQueue()}
     */
    public boolean getHasDetectedNote() {
        //System.out.printf("%s -> %s\n", colorSensorTarget.getTopic().getName(), colorSensorTarget.getAsBoolean());
        boolean[] values = colorSensorTarget.readQueueValues();

        for (boolean val : values) {
            if (val) return true;
        }

        return false;
    }

    @Override
    public void periodic(){
        indexerVeloLog.append(indexerEncoder.getVelocity() * ShooterConstants.kINDEXER_RATIO);
    }
}
