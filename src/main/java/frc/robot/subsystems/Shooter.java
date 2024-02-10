package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.frc7153.diagnostics.DiagUtil;

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

    // Control
    private VelocityVoltage shooterControl = new VelocityVoltage(0.0).withSlot(0);
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

        // Init sensors
        NetworkTable sensorTable = NetworkTableInstance.getDefault().getTable("SecondaryPiSensors");
        leftColorSensorTarget = sensorTable.getBooleanTopic("LeftTarget").subscribe(false);
        rightColorSensorTarget = sensorTable.getBooleanTopic("RightTarget").subscribe(false);

        // Begin running diagnostics on these motors
        DiagUtil.addDevice(shooterUpper);
        DiagUtil.addDevice(shooterLower);

        // Initial log value
        shooterSetpointLog.append(0.0);

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

    }
}
