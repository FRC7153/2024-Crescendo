package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.frc7153.diagnostics.DiagUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShooterConstants;

/**
 * Shooter and indexer of robot
 */
public class Shooter implements Subsystem {
    // Devices
    private TalonFX shooterUpper = new TalonFX(HardwareConstants.kSHOOTER_UPPER_CAN, HardwareConstants.kCANIVORE_BUS);
    private TalonFX shooterLower = new TalonFX(HardwareConstants.kSHOOTER_LOWER_CAN, HardwareConstants.kCANIVORE_BUS);
    private CANSparkMax indexer = new CANSparkMax(HardwareConstants.kINDEXER_CAN, MotorType.kBrushless);
    private RelativeEncoder indexerEncoder = indexer.getEncoder();

    // Control
    private VelocityVoltage shooterControl = new VelocityVoltage(0.0).withSlot(0);
    private SparkPIDController indexerControl;
    private double velocitySetpoint = 0.0;

    // Sensors
    private BooleanSubscriber leftColorSensorTarget;
    private BooleanSubscriber rightColorSensorTarget;

    // Logging
    private DoubleLogEntry shooterSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Shooter/Setpoint Velocity", "rps");
    private DoubleLogEntry upperShooterVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Shooter/Upper Velocity", "rps");
    private DoubleLogEntry lowerShooterVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Shooter/Lower Velocity", "rps");
    private DoubleLogEntry indexerSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Indexer/Setpoint Velocity", "rps");
    private DoubleLogEntry indexerVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Indexer/Velocity", "rps");

    // Init
    public Shooter() {
        // Create config
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.Slot0.kP = ShooterConstants.kSHOOT_P;
        shooterConfig.Slot0.kI = ShooterConstants.kSHOOT_I;
        shooterConfig.Slot0.kD = ShooterConstants.kSHOOT_D;

        shooterConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.kSHOOT_CURRENT_LIMIT;
        shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Config shooter motors
        shooterUpper.getConfigurator().apply(shooterConfig);
        shooterLower.getConfigurator().apply(shooterConfig);

        shooterUpper.setInverted(true);
        shooterLower.setInverted(true);

        shooterUpper.setControl(shooterControl);
        shooterLower.setControl(shooterControl);

        // Config indexer motor
        indexer.setIdleMode(IdleMode.kBrake);
        indexer.setInverted(false);
        indexer.setSmartCurrentLimit(ShooterConstants.kINDEXER_CURRENT_LIMIT);

        indexerControl = indexer.getPIDController();
        indexerControl.setP(ShooterConstants.kINDEXER_P, 0);
        indexerControl.setI(ShooterConstants.kINDEXER_I, 0);
        indexerControl.setD(ShooterConstants.kINDEXER_D, 0);

        // Init sensors
        NetworkTable sensorTable = NetworkTableInstance.getDefault().getTable("SecondaryPiSensors");
        leftColorSensorTarget = sensorTable.getBooleanTopic("LeftTarget").subscribe(false);
        rightColorSensorTarget = sensorTable.getBooleanTopic("RightTarget").subscribe(false);

        // Begin running diagnostics on these motors
        DiagUtil.addDevice(shooterUpper);
        DiagUtil.addDevice(shooterLower);
        DiagUtil.addDevice(indexer);

        // Initial log values
        shooterSetpointLog.append(0.0);
        indexerSetpointLog.append(0.0);

        // Register
        register();
    }

    /** Set default command (not moving) */
    public void setDefaultCommand() {
        // Alex: ignore this for now, remind me about it Saturday :)
        /*setDefaultCommand(new InstantCommand(() -> {
            setShootVelocity(0.0);
            setIndexerVelocity(0.0);
        }, this));*/
    }

    /** Set shoot velocity (r/s) */
    public void setShootVelocity(double velocity) {

        if (velocity <= .05) {
            shooterUpper.disable();
            shooterLower.disable();
        } else {
            shooterUpper.setControl(shooterControl.withVelocity(velocity / ShooterConstants.kSHOOT_RATIO));
            shooterLower.setControl(shooterControl.withVelocity(velocity / ShooterConstants.kSHOOT_RATIO));
        }

        shooterSetpointLog.append(velocity);
        velocitySetpoint = velocity;
    }

    /** Sets indexer enabled (r/m) */
    public void setIndexerVelocity(double velocity) {
        indexerControl.setReference(velocity / ShooterConstants.kINDEXER_RATIO, ControlType.kVelocity, 0);
        indexerSetpointLog.append(ShooterConstants.kINDEXER_SETPOINT);
    }

    /** Is the shooter velocity at the setpoint? */
    public boolean atShootSetpoint() {
        return Math.abs(shooterUpper.getVelocity().getValue() - velocitySetpoint) <= ShooterConstants.kSHOOT_TOLERANCE &&
            Math.abs(shooterLower.getVelocity().getValue() - velocitySetpoint) <= ShooterConstants.kSHOOT_TOLERANCE;
    }

    // Perform logging
    @Override
    public void periodic() {
        upperShooterVeloLog.append(shooterUpper.getVelocity().getValue() * ShooterConstants.kSHOOT_RATIO);
        lowerShooterVeloLog.append(shooterLower.getVelocity().getValue() * ShooterConstants.kSHOOT_RATIO);

        indexerVeloLog.append(indexerEncoder.getVelocity() * ShooterConstants.kINDEXER_RATIO);
    }
}
