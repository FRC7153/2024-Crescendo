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

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ArmConstants;

public class Arm implements Subsystem {
    /** Class for representing a state of the arm */
    public static class ArmState {
			public double lowerAngle; // deg
			public double upperAngle; // deg
			public double ext; // rots

			public ArmState(double lowerAngle, double upperAngle, double ext) {
				this.lowerAngle = lowerAngle;
				this.upperAngle = upperAngle;
				this.ext = ext;
			}
    }

    // Hardware
    private CANSparkMax lowerLeftPivot = new CANSparkMax(HardwareConstants.kLOWER_LEFT_PIVOT_CAN, MotorType.kBrushless);//has absolute encoder
    private CANSparkMax lowerRightPivot = new CANSparkMax(HardwareConstants.kLOWER_RIGHT_PIVOT_CAN, MotorType.kBrushless);
    private CANSparkMax elevatorExt = new CANSparkMax(HardwareConstants.kELEVATOR_EXT_CAN, MotorType.kBrushless);
    private CANSparkMax upperPivot = new CANSparkMax(HardwareConstants.kUPPER_PIVOT_CAN, MotorType.kBrushless);//has absolute encoder

    private SparkAbsoluteEncoder lowerPivotEncoder = lowerLeftPivot.getAbsoluteEncoder(Type.kDutyCycle); // TODO switched?
    private RelativeEncoder elevatorExtEncoder = elevatorExt.getEncoder();
    private SparkLimitSwitch elevatorLimitSwitch = elevatorExt.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    private SparkAbsoluteEncoder upperPivotEncoder = upperPivot.getAbsoluteEncoder(Type.kDutyCycle);

    // Control
    private SparkPIDController elevatorExtController;
    private SparkPIDController lowerLeftPivotController;
    private SparkPIDController upperPivotController; 

    private ArmState setpoint = new ArmState(0.0, 0.0, 0.0);

    private DoubleLogEntry lowerPivotPositionLog = 
			new DoubleLogEntry(DataLogManager.getLog(), "Arm/lowerPivotPosition", "deg");
    private DoubleLogEntry elevatorExtPositionLog = 
			new DoubleLogEntry(DataLogManager.getLog(), "Arm/ElevatorExtPosition", "rotations");
    private DoubleLogEntry upperPivotPositionLog = 
			new DoubleLogEntry(DataLogManager.getLog(), "Arm/upperPivotPosition", "deg");
		private BooleanLogEntry elevatorLimitSwitchLog = 
			new BooleanLogEntry(DataLogManager.getLog(), "Arm/elevatorLimitSwitch", "pressed?");
    private DoubleLogEntry lowerPivotSetpointLog = 
			new DoubleLogEntry(DataLogManager.getLog(), "Arm/lowerPivotSetpoint", "deg");
    private DoubleLogEntry upperPivotSetpointLog = 
			new DoubleLogEntry(DataLogManager.getLog(), "Arm/upperPivotSetpoint", "deg");
    private DoubleLogEntry elevatorExtSetpointLog = 
			new DoubleLogEntry(DataLogManager.getLog(), "Arm/elevatorExtSetpoint", "rotations");


    public Arm() {
        // Config motors
        lowerLeftPivot.setIdleMode(IdleMode.kBrake);
        lowerRightPivot.setIdleMode(IdleMode.kBrake);
        upperPivot.setIdleMode(IdleMode.kBrake);
        elevatorExt.setIdleMode(IdleMode.kBrake);

        lowerLeftPivot.setInverted(false);
        lowerRightPivot.setInverted(false);
        upperPivot.setInverted(false);
        elevatorExt.setInverted(false);

        lowerRightPivot.follow(lowerLeftPivot, true);

        // Config PID
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

        // Config sensors
        elevatorLimitSwitch.enableLimitSwitch(true);

        // TODO verify polarity
        lowerPivotEncoder.setInverted(false);
        lowerPivotEncoder.setZeroOffset(ArmConstants.kLOWER_ANGLE_OFFSET);

        upperPivotEncoder.setInverted(false);
        upperPivotEncoder.setZeroOffset(ArmConstants.kUPPER_ANGLE_OFFSET);

        lowerLeftPivotController.setFeedbackDevice(lowerPivotEncoder);
        upperPivotController.setFeedbackDevice(upperPivotEncoder);

        //config logging 
        DiagUtil.addDevice(lowerLeftPivot);
        DiagUtil.addDevice(lowerRightPivot);
        DiagUtil.addDevice(upperPivot);
        DiagUtil.addDevice(elevatorExt);

        register();
    }

    /** Sets Arm's default command (laying flat) */
    public void initDefaultCommand() {
        setDefaultCommand(
            new InstantCommand(() -> setState(
                new ArmState(0.0, 0.0, 0.0)
            ), this)
        );
    }

    /**
     * Sets the lower pivot's setpoint
     * @param angle degrees. 0 is down, 90 is up
     */
    public void setLowerPivotAngle(double angle) {
        // Safety
        angle = Math.max(0.0, Math.min(angle, 100.0));

        lowerLeftPivotController.setReference((angle / 360.0) / ArmConstants.kLOWER_PIVOT_RATIO, ControlType.kPosition);

        setpoint.lowerAngle = angle;

        lowerPivotSetpointLog.append(angle);
    }

    /**
     * Sets the upper pivot's setpoint
     * @param angle degrees. 0 is forward, positive is up
     */
    public void setUpperPivotAngle(double angle) {
        // Safety
        angle = Math.max(-180.0, Math.min(angle, 180.0));

        upperPivotController.setReference((angle / 360.0) / ArmConstants.kUPPER_PIVOT_RATIO, ControlType.kPosition);

        setpoint.upperAngle = angle;

        upperPivotSetpointLog.append(angle);
    }

    /**
     * Sets the elevator's extension
     * @param rots rotations. 0 is inward
     */
    public void setExtension(double rots) {
        // Safety
        rots = Math.max(0.0, Math.min(rots, 20.0)); // TODO max rots

        elevatorExtController.setReference(rots / ArmConstants.kELEVATOR_EXT_RATIO , ControlType.kPosition);

        setpoint.ext = rots;

        elevatorExtSetpointLog.append(rots);
    }

    public void setState(ArmState state){
        setUpperPivotAngle(state.upperAngle);
        setLowerPivotAngle(state.lowerAngle);
        setExtension(state.ext);
    }

    /**
     * If the arm is within tolerance of the setpoints
     */
    public boolean atSetpoint() {
        return Math.abs((lowerPivotEncoder.getPosition() * 360.0) - setpoint.lowerAngle) <= ArmConstants.kLOWER_ANGLE_TOLERANCE &&
            Math.abs((upperPivotEncoder.getPosition() * 360.0) - setpoint.upperAngle) <= ArmConstants.kUPPER_ANGLE_TOLERANCE &&
            Math.abs((elevatorExtEncoder.getPosition() * ArmConstants.kELEVATOR_EXT_RATIO) - setpoint.ext) <= ArmConstants.kEXT_TOLERANCE;
    }

    @Override
    public void periodic(){
        lowerPivotPositionLog.append(lowerPivotEncoder.getPosition() * 360.0 * ArmConstants.kLOWER_PIVOT_RATIO);

        upperPivotPositionLog.append(upperPivotEncoder.getPosition() * 360.0 * ArmConstants.kUPPER_PIVOT_RATIO);
        
        elevatorExtPositionLog.append(elevatorExtEncoder.getPosition() * ArmConstants.kELEVATOR_EXT_RATIO);
				elevatorLimitSwitchLog.append(elevatorLimitSwitch.isPressed());
    }

    // TEST MODE //
    private SimpleWidget testLowerPivot;
    private SimpleWidget testUpperPivot;
    private SimpleWidget testExt;

    /** Initializes shuffleboard values */
    public void initTestMode() {
        ShuffleboardTab tab = Shuffleboard.getTab("Arm Debug");
        
        testLowerPivot = tab.add("Lower Pivot", 0.0)
            .withPosition(0, 0);

        testUpperPivot = tab.add("Upper Pivot", 0.0)
            .withPosition(1, 0);

        testExt = tab.add("Ext", 0.0)
            .withPosition(2, 0);
    }

    /** Runs test mode */
    public void execTestMode() {
        setUpperPivotAngle(testUpperPivot.getEntry().getDouble(0.0));
        setLowerPivotAngle(testLowerPivot.getEntry().getDouble(0.0));
        setExtension(testExt.getEntry().getDouble(0.0));

        periodic();
    }

    /** Stops test mode */
    public void endTestMode() {
        testUpperPivot.close();
        testLowerPivot.close();
        testExt.close();

        testUpperPivot = null;
        testLowerPivot = null;
        testExt = null;
    }
}
