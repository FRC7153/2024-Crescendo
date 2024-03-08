package frc.robot.subsystems;


import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;

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

    // Logging
    private DoubleLogEntry intakeSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Intake/Setpoint", "rpm");
    private DoubleLogEntry intakeVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Intake/Velocity", "rpm");

    // Init
    public Intake() {
        intake.setIdleMode(IdleMode.kBrake);
        intake.setInverted(false);
        intake.setSmartCurrentLimit(IntakeConstants.kINTAKE_CURRENT_LIMIT);

        intakeController = intake.getPIDController();
        intakeController.setP(IntakeConstants.kINTAKE_P, 0);
        intakeController.setI(IntakeConstants.kINTAKE_I, 0);
        intakeController.setD(IntakeConstants.kINTAKE_D, 0);

        // Config logging
        DiagUtil.addDevice(intake);
        intakeSetpointLog.append(0.0);

        register();
    }

    /** Sets the intake's default command (not moving) */
    public void initDefaultCommand() {
        setDefaultCommand(new InstantCommand(
            this::end, this
        ));
    }

    /** Runs the intake forward */
    public void enableIntake() {
        //intakeController.setReference(4000, ControlType.kVelocity, 0);
        //intakeSetpointLog.append(4000 * IntakeConstants.kINTAKE_RATIO);
        intake.set(0.92);
    }

    /** Runs the intake in reverse */
    public void reverseIntake() {
        intakeController.setReference(-4000, ControlType.kVelocity, 0);
        intakeSetpointLog.append(-4000 * IntakeConstants.kINTAKE_RATIO);
    }

    /** Ends the intake function */
    public void end() {
        intakeController.setReference(0.0, ControlType.kVelocity, 0);
        intakeSetpointLog.append(0.0);
    }

    @Override
    public void periodic() {
        intakeVeloLog.append(intakeEncoder.getVelocity() * IntakeConstants.kINTAKE_RATIO);
    }
}
