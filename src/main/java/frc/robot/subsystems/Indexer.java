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
    private BooleanSubscriber leftColorSensorTarget;
    private BooleanSubscriber rightColorSensorTarget;

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
        indexerControl.setP(ShooterConstants.kINDEXER_P, 0);
        indexerControl.setI(ShooterConstants.kINDEXER_I, 0);
        indexerControl.setD(ShooterConstants.kINDEXER_D, 0);

        // Init sensors
        NetworkTable sensorTable = NetworkTableInstance.getDefault().getTable("SecondaryPiSensors");
        leftColorSensorTarget = sensorTable.getBooleanTopic("LeftTarget").subscribe(false);
        rightColorSensorTarget = sensorTable.getBooleanTopic("RightTarget").subscribe(false);

        DiagUtil.addDevice(indexer);
        DiagUtil.addDevice(leftColorSensorTarget.getTopic());
        DiagUtil.addDevice(rightColorSensorTarget.getTopic());

        indexerSetpointLog.append(0.0);

        register();
    }
    /**
     * Sets velocity in rpm
     * @param velocity
     */
    public void setIndexerVelocity(double velocity) {
        indexerControl.setReference(velocity / ShooterConstants.kINDEXER_RATIO, ControlType.kVelocity, 0);
        indexerSetpointLog.append(ShooterConstants.kINDEXER_SETPOINT);
    }

    public void setDefaultCommand(){
        setDefaultCommand(new InstantCommand(() -> setIndexerVelocity(0.0), this));
    }

    /**
     * @return If either the left or right sensor sees the piece
     */
    public boolean detectingNote() {
        return leftColorSensorTarget.get() || rightColorSensorTarget.get();
    }

    @Override
    public void periodic(){
        indexerVeloLog.append(indexerEncoder.getVelocity() * ShooterConstants.kINDEXER_RATIO);
    }
}
