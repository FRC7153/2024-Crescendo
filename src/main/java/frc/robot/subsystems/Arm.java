package frc.robot.subsystems;

import com.frc7153.diagnostics.DiagUtil;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ArmConstants;

public class Arm implements Subsystem {
    /** Class for representing a state of the arm */
    public static class ArmState {
        public final double lowerAngle; // deg
        public final double upperAngle; // deg
        public final double ext; // rots

        public ArmState(double lowerAngle, double upperAngle, double ext) {
            this.lowerAngle = lowerAngle;
            this.upperAngle = upperAngle;
            this.ext = ext;
        }
    }

    private CANSparkMax lowerLeftPivot = new CANSparkMax(HardwareConstants.kLOWER_LEFT_PIVOT_CAN, MotorType.kBrushless);//has relative encoder
    private CANSparkMax lowerRightPivot = new CANSparkMax(HardwareConstants.kLOWER_RIGHT_PIVOT_CAN, MotorType.kBrushless);
    private CANSparkMax elevatorExt = new CANSparkMax(HardwareConstants.kELEVATOR_EXT_CAN, MotorType.kBrushless);
    private CANSparkMax upperPivot = new CANSparkMax(HardwareConstants.kUPPER_PIVOT_CAN, MotorType.kBrushless);//has relative encoder

    private SparkAbsoluteEncoder lowerLeftPivotEncoder = lowerLeftPivot.getAbsoluteEncoder(Type.kDutyCycle);
    private RelativeEncoder elevatorExtEncoder = elevatorExt.getEncoder();
    private SparkLimitSwitch elevatorLimitSwitch = elevatorExt.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    private AbsoluteEncoder upperPivotEncoder = upperPivot.getAbsoluteEncoder(Type.kDutyCycle);

    private SparkPIDController elevatorExtController;
    private SparkPIDController lowerLeftPivotController;
    private SparkPIDController upperPivotController; 

    private ArmState setpoint = new ArmState(0.0, 0.0, 0.0);

    private DoubleLogEntry lowerLeftPivotPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Arm/lowerLeftPivotPosition", "deg");
    private DoubleLogEntry lowerRightPivotPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Arm/lowerRightPivotPosition", "deg");
    private DoubleLogEntry elevatorExtPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Arm/ElevatorExtPosition", "rotations");
    private DoubleLogEntry upperPivotPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Arm/upperPivotPosition", "deg");
    private DoubleLogEntry lowerLeftPivotSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Arm/lowerPivotSetpoint", "deg");
    private DoubleLogEntry upperPivotSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Arm/upperPivotSetpoint", "deg");
    private DoubleLogEntry elevatorExtSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Arm/elevatorExtSetpoint", "rotations");


    public Arm() {
        lowerLeftPivot.setIdleMode(IdleMode.kBrake);
        lowerRightPivot.setIdleMode(IdleMode.kBrake);
        upperPivot.setIdleMode(IdleMode.kBrake);
        elevatorExt.setIdleMode(IdleMode.kBrake);

        lowerLeftPivot.setInverted(false);
        lowerRightPivot.setInverted(false);
        upperPivot.setInverted(false);
        elevatorExt.setInverted(false);
        
       elevatorExtController = elevatorExt.getPIDController();
       lowerLeftPivotController = lowerLeftPivot.getPIDController();
       upperPivotController = upperPivot.getPIDController();

       elevatorExtController.setP(ArmConstants.kELEVATOR_EXT_P, 0);
       elevatorExtController.setI(ArmConstants.kELEVATOR_EXT_I, 0);
       elevatorExtController.setD(ArmConstants.kELEVATOR_EXT_D, 0);

       lowerLeftPivotController.setP(ArmConstants.kLOWER_PIVOT_P, 0);
       lowerLeftPivotController.setI(ArmConstants.kLOWER_PIVOT_I, 0);
       lowerLeftPivotController.setD(ArmConstants.kLOWER_PIVOT_D, 0);

       upperPivotController.setP(ArmConstants.kUPPER_PIVOT_P, 0);
       upperPivotController.setI(ArmConstants.kUPPER_PIVOT_I, 0);
       upperPivotController.setD(ArmConstants.kUPPER_PIVOT_D, 0);

       elevatorLimitSwitch.enableLimitSwitch(true);

       lowerRightPivot.follow(lowerLeftPivot, true);

       //config logging 
       DiagUtil.addDevice(lowerLeftPivot);
       DiagUtil.addDevice(lowerRightPivot);
       DiagUtil.addDevice(upperPivot);
       DiagUtil.addDevice(elevatorExt);

       register();
    }

    /**
     * Sets the lower pivot's setpoint
     * @param angle degrees. 0 is down, 90 is up
     */
    public void setLowerPivotAngle(double angle) {
        lowerLeftPivotController.setReference((angle / 360.0) / ArmConstants.kLOWER_PIVOT_RATIO, ControlType.kPosition);

        lowerLeftPivotSetpointLog.append(angle);
    }

    /**
     * Sets the upper pivot's setpoint
     * @param angle degrees. 0 is forward, positive is up
     */
    public void setUpperPivotAngle(double angle) {
        upperPivotController.setReference((angle / 360.0) / ArmConstants.kUPPER_PIVOT_RATIO, ControlType.kPosition);

        upperPivotSetpointLog.append(angle);
    }

    /**
     * Sets the elevator's extension
     * @param rots rotations. 0 is inward
     */
    public void setExtension(double rots) {
        elevatorExtController.setReference(rots / ArmConstants.kELEVATOR_EXT_RATIO , ControlType.kPosition);

        elevatorExtSetpointLog.append(rots);
    }

    public boolean lowerPivotAtSetpoint(){
        return false;
    }

    public boolean upperPivotAtSetpoint(){
        return false;
    }

    public boolean extensionAtSetpoint(){
        return false;
    }

    @Override
    public void periodic(){
        lowerLeftPivotPositionLog.append(lowerLeftPivotEncoder.getPosition() * 360.0 * ArmConstants.kLOWER_PIVOT_RATIO);
        lowerRightPivotPositionLog.append(lowerLeftPivotEncoder.getPosition() * 360.0 * ArmConstants.kLOWER_PIVOT_RATIO);

        upperPivotPositionLog.append(upperPivotEncoder.getPosition() * 360.0 * ArmConstants.kUPPER_PIVOT_RATIO);
        
        elevatorExtPositionLog.append(elevatorExtEncoder.getPosition() * ArmConstants.kELEVATOR_EXT_RATIO);
    }
}
