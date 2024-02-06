package frc.robot.subsystems;

import com.frc7153.diagnostics.DiagUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ArmConstants;

public class Arm implements Subsystem {
    private CANSparkMax lowerLeftPivot = new CANSparkMax(HardwareConstants.kLOWER_LEFT_PIVOT_CAN, MotorType.kBrushless);//has relative encoder
    private CANSparkMax lowerRightPivot = new CANSparkMax(HardwareConstants.kLOWER_RIGHT_PIVOT_CAN, MotorType.kBrushless);
    private CANSparkMax elevatorExt = new CANSparkMax(HardwareConstants.kELEVATOR_EXT_CAN, MotorType.kBrushless);
    private CANSparkMax upperPivot = new CANSparkMax(HardwareConstants.kUPPER_PIVOT_CAN, MotorType.kBrushless);//has relative encoder

    private SparkAbsoluteEncoder lowerLeftPivotEncoder = lowerLeftPivot.getAbsoluteEncoder(Type.kDutyCycle);
    private SparkLimitSwitch elevatorLimitSwitch = elevatorExt.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    private RelativeEncoder upperPivotEncoder = upperPivot.getEncoder();

    private SparkPIDController elevatorExtController;
    private SparkPIDController lowerLeftPivotController;
    private SparkPIDController upperPivotController; 

    private DoubleLogEntry lowerLeftPivotPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "lowerLeftPivot/Position", "rotations");
    private DoubleLogEntry lowerRightPivotPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "lowerRightPivot/Position", "rotations");
    private DoubleLogEntry elevatorExtPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "elevatorExt/Position", "rotations");
    private DoubleLogEntry upperPivotPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "upperPivot/Position", "rotations");
    private DoubleLogEntry lowerLeftPivotSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "lowerLeftPivot/Setpoint", "rotations");
    private DoubleLogEntry lowerRightPivotSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "lowerRightPivot/Setpoint", "rotations");
    private DoubleLogEntry upperPivotSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "upperPivot/Setpoint", "rotations");
    private DoubleLogEntry elevatorExtSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "elevatorExt/Setpoint", "rotations");


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
    }

    public void setUpperPivotAngle(double angle) {
        upperPivotController.setReference((angle / 360.0) / ArmConstants.kUPPER_PIVOT_RATIO, ControlType.kPosition);

    }

    public void setExtension(double inches) {
        elevatorExtController.setReference((inches * 360) * ArmConstants.kELEVATOR_EXT_RATIO , ControlType.kPosition);
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

    }
}
