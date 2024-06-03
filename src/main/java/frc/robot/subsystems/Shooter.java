package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.frc7153.diagnostics.DiagUtil;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.BuildConstants;
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

    // Logging
    private DoubleLogEntry shooterSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Shooter/Setpoint Velocity", "rps");
    private DoubleLogEntry upperShooterVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Shooter/Upper Velocity", "rps");
    private DoubleLogEntry lowerShooterVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Shooter/Lower Velocity", "rps");

    // Telemetry
    private GenericPublisher upperShootVeloOut, lowerShootVeloOut, shootAtSetpointOut;

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

        // Begin running diagnostics on these motors
        DiagUtil.addDevice(shooterUpper);
        DiagUtil.addDevice(shooterLower);

        // Initial log value
        shooterSetpointLog.append(0.0);

        // Telemetry
        if (BuildConstants.kOUTPUT_ALL_TELEMETRY) {
            ShuffleboardTab tab = Shuffleboard.getTab("Shooter Telemetry");

            upperShootVeloOut = tab.add("Upper Shoot Velo (RPS)", -1.0)
                .getEntry().getTopic().genericPublish("double");
            
            lowerShootVeloOut = tab.add("Lower Shoot Velo (RPS)", -1.0)
                .getEntry().getTopic().genericPublish("double");

            shootAtSetpointOut = tab.add("At Setpoint", false)
                .getEntry().getTopic().genericPublish("boolean");
        }

        // Register
        register();
    }

    /** Set default command (not moving) */
    public void initDefaultCommand() {
        setDefaultCommand(new InstantCommand(() -> {
            setShootVelocity(0.0);
        }, this));
    }

    /** Set shoot velocity (r/s) */
    public void setShootVelocity(double velocity) {
        if (Math.abs(velocity) <= .05) {
            shooterUpper.disable();
            shooterLower.disable();

            shooterSetpointLog.append(0.0);
            velocitySetpoint = 0.0;
        } else {
            shooterUpper.setControl(shooterControl.withVelocity(velocity / ShooterConstants.kSHOOT_RATIO));
            shooterLower.setControl(shooterControl.withVelocity(velocity / ShooterConstants.kSHOOT_RATIO));

            shooterSetpointLog.append(velocity);
            velocitySetpoint = velocity;
        }
    }

    public void setShootVelocity(double lowerShootVelocity, double upperShootVelocity) {
        if (Math.abs(lowerShootVelocity) <= .05 && Math.abs(upperShootVelocity) <= .05) {
            shooterUpper.disable();
            shooterLower.disable();

            velocitySetpoint = 0.0;
        } else {
            shooterUpper.setControl(shooterControl.withVelocity(upperShootVelocity / ShooterConstants.kSHOOT_RATIO));
            shooterLower.setControl(shooterControl.withVelocity(lowerShootVelocity / ShooterConstants.kSHOOT_RATIO));

            velocitySetpoint = upperShootVelocity;
        }
    }

    /** Is the shooter velocity at the setpoint? */
    public boolean atShootSetpoint() {
        return Math.abs(shooterUpper.getVelocity().getValue() - velocitySetpoint) <= ShooterConstants.kSHOOT_TOLERANCE &&
            Math.abs(shooterLower.getVelocity().getValue() - velocitySetpoint) <= ShooterConstants.kSHOOT_TOLERANCE;
    }

    /** Is the shooter setpoint > 0.0? */
    public boolean isShooterRunning() {
        return velocitySetpoint > 0.0;
    }

    // Perform logging
    @Override
    public void periodic() {
        upperShooterVeloLog.append(shooterUpper.getVelocity().getValue() * ShooterConstants.kSHOOT_RATIO);
        lowerShooterVeloLog.append(shooterLower.getVelocity().getValue() * ShooterConstants.kSHOOT_RATIO);

        // Telemetry
        if (BuildConstants.kOUTPUT_ALL_TELEMETRY) {
            upperShootVeloOut.setDouble(shooterUpper.getVelocity().getValue() * ShooterConstants.kSHOOT_RATIO);
            lowerShootVeloOut.setDouble(shooterLower.getVelocity().getValue() * ShooterConstants.kSHOOT_RATIO);
            shootAtSetpointOut.setBoolean(atShootSetpoint());
        }
    }

    // TEST MODE //
    private GenericEntry lowerTestShootVelo, upperTestShootVelo;

    public void testInit() {
        if (lowerTestShootVelo != null) return; // Already initialized

        ShuffleboardTab tab = Shuffleboard.getTab("Arm Debug");

        lowerTestShootVelo = tab.add("Lower Shoot Velocity (RPS)", 0.0)
            .getEntry();
        
        upperTestShootVelo = tab.add("Upper Shoot Velocity (RPS)", 0.0)
            .getEntry();
    }

    public void testExec(CommandJoystick operatorController) {
        if (operatorController.button(2).getAsBoolean()) {
            // Left button 2 down
            setShootVelocity(
                lowerTestShootVelo.getDouble(0.0), 
                upperTestShootVelo.getDouble(0.0)
            );
        } else {
            setShootVelocity(0.0);
        }
    }

    public void testEnd() {
        setShootVelocity(0.0);
    }
}
