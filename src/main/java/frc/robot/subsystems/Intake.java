package frc.robot.subsystems;


import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SecondaryIntakeConstants;
import frc.robot.util.Util;

import com.frc7153.diagnostics.DiagUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** Lower Intake Subsystem */
public class Intake implements Subsystem {
    // Hardware
    private CANSparkMax intake = new CANSparkMax(HardwareConstants.kINTAKE_CAN, MotorType.kBrushless);
    private SparkPIDController intakeController;
    private RelativeEncoder intakeEncoder = intake.getEncoder();

    private CANSparkMax secondaryIntake = new CANSparkMax(HardwareConstants.kINTAKE_SECONDARY, MotorType.kBrushless);
    private SparkPIDController secondaryIntakeController;
    private RelativeEncoder secondaryIntakeEncoder = intake.getEncoder();

    // Setpoint
    private double setpoint = 0.0;
    private double setpointTimestamp = 0.0;

    // Logging
    private DoubleLogEntry intakeSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Intake/Setpoint", "rpm");
    private DoubleLogEntry intakeVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Intake/Velocity", "rpm");

    private DoubleLogEntry secondarySetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "SecondaryIntake/Setpoint", "rpm");
    private DoubleLogEntry secondaryIntakeVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "SecondaryIntake/Setpoint", "rpm");

    // Init
    public Intake() {
        // Clear some problematic configs
        intake.restoreFactoryDefaults();

        secondaryIntake.setIdleMode(IdleMode.kBrake);
        secondaryIntake.setInverted(false);
        secondaryIntake.setSmartCurrentLimit(SecondaryIntakeConstants.kINTAKE_CURRENT_LIMIT);

        intake.setIdleMode(IdleMode.kBrake);
        intake.setInverted(false);
        intake.setSmartCurrentLimit(IntakeConstants.kINTAKE_CURRENT_LIMIT);

        secondaryIntakeController = secondaryIntake.getPIDController();
        secondaryIntakeController.setP(SecondaryIntakeConstants.kINTAKE_P, 0);
        secondaryIntakeController.setI(SecondaryIntakeConstants.kINTAKE_I, 0);
        secondaryIntakeController.setD(SecondaryIntakeConstants.kINTAKE_D, 0);

        intakeController = intake.getPIDController();
        intakeController.setP(IntakeConstants.kINTAKE_P, 0);
        intakeController.setI(IntakeConstants.kINTAKE_I, 0);
        intakeController.setD(IntakeConstants.kINTAKE_D, 0);

        // Config logging
        DiagUtil.addDevice(intake);
        intakeSetpointLog.append(0.0);

        DiagUtil.addDevice(secondaryIntake);
        secondarySetpointLog.append(0.0);

        register();

        // Reduce CAN usage
        Util.disableExternalEncoderFrames(intake);
        Util.disableExternalEncoderFrames(secondaryIntake);
    }

    /** Sets the intake's default command (not moving) */
    public void initDefaultCommand() {
        setDefaultCommand(new InstantCommand(
            this::end, this
        ));
    }
    /** Runs the Intake Motors at the Maximum speed */
    private void intakeFullSend() {
        setpoint = 0.0; // no setpoint
        setpointTimestamp = Timer.getFPGATimestamp();

        secondaryIntake.set(1.0);
        secondarySetpointLog.append(6000 * SecondaryIntakeConstants.kINTAKE_RATIO);

        intake.set(1.0);
        intakeSetpointLog.append(6000 * IntakeConstants.kINTAKE_RATIO);
    }

    /** Runs the intake at a speed */
    private void setIntakeSpeed(double velocity) {
        setpointTimestamp = Timer.getFPGATimestamp();
        setpoint = velocity;
        
        secondaryIntakeController.setReference(velocity, ControlType.kVelocity, 0);
        secondarySetpointLog.append(velocity * SecondaryIntakeConstants.kINTAKE_RATIO);

        intakeController.setReference(velocity, ControlType.kVelocity, 0);
        intakeSetpointLog.append(velocity * IntakeConstants.kINTAKE_RATIO);
    }

    /** Runs the intake forward */
    public void enableIntake() {
        intakeFullSend();
    }

    /** Runs the intake in reverse */
    public void reverseIntake() {
        setIntakeSpeed(-5000);
    }

    /** Ends the intake function */
    public void end() {
        setIntakeSpeed(0.0);
        intake.disable();
        secondaryIntake.disable();
    }

    @Override
    public void periodic() {
        intakeVeloLog.append(intakeEncoder.getVelocity() * IntakeConstants.kINTAKE_RATIO);
        secondaryIntakeVeloLog.append(secondaryIntakeEncoder.getVelocity() * SecondaryIntakeConstants.kINTAKE_RATIO);

        // Ensure not stalled
        if (
            Timer.getFPGATimestamp() - setpointTimestamp >= 3.5 && // Time has passed since setpoint
            Math.abs(setpoint) > 1.0 && // Setpoint is not 0
            Math.abs(secondaryIntakeEncoder.getVelocity()) < 100.0 && // Speed is very low
            Math.abs(intakeEncoder.getVelocity()) < 100.0 // Speed is very low
        ) {
            DriverStation.reportError("Intake motor PID stall detected!", false);
            intake.set(setpoint / 6000.0);
            secondaryIntake.set(setpoint / 6000.00);
        }
    }
}
