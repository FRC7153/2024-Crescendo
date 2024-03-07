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
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HardwareConstants;
import frc.robot.commands.ArmToStateCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPositions;

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

    private SparkAbsoluteEncoder lowerPivotEncoder = lowerRightPivot.getAbsoluteEncoder(Type.kDutyCycle);
    private RelativeEncoder elevatorExtEncoder = elevatorExt.getEncoder();
    private SparkLimitSwitch elevatorLimitSwitch = elevatorExt.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    private SparkAbsoluteEncoder upperPivotEncoder = upperPivot.getAbsoluteEncoder(Type.kDutyCycle);
    private RelativeEncoder upperPivotRelEncoder = upperPivot.getEncoder();

    // Control
    private SparkPIDController elevatorExtController;
    private SparkPIDController lowerRightPivotController;
    private SparkPIDController upperPivotController; 

    private ArmState setpoint;

    private DoubleLogEntry lowerPivotPositionLog = 
		new DoubleLogEntry(DataLogManager.getLog(), "Arm/lowerPivotPosition", "deg");
    private DoubleLogEntry elevatorExtPositionLog = 
		new DoubleLogEntry(DataLogManager.getLog(), "Arm/ElevatorExtPosition", "rotations");
    private DoubleLogEntry upperPivotPositionLog = 
		new DoubleLogEntry(DataLogManager.getLog(), "Arm/upperPivotPosition", "deg");
    private BooleanLogEntry upperPivotSafeToMoveLog = 
        new BooleanLogEntry(DataLogManager.getLog(), "Arm/upperPivotSafeToMove");
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

        lowerLeftPivot.setInverted(true);
        lowerRightPivot.setInverted(false);
        upperPivot.setInverted(false);
        elevatorExt.setInverted(true);

        lowerLeftPivot.follow(lowerRightPivot, true);

        // Config PID
        elevatorExtController = elevatorExt.getPIDController();
        lowerRightPivotController = lowerRightPivot.getPIDController();
        upperPivotController = upperPivot.getPIDController();

        elevatorExtController.setP(ArmConstants.kELEVATOR_EXT_P, 0);
        elevatorExtController.setI(ArmConstants.kELEVATOR_EXT_I, 0);
        elevatorExtController.setD(ArmConstants.kELEVATOR_EXT_D, 0);

        lowerRightPivotController.setP(ArmConstants.kLOWER_PIVOT_P, 0);
        lowerRightPivotController.setI(ArmConstants.kLOWER_PIVOT_I, 0);
        lowerRightPivotController.setD(ArmConstants.kLOWER_PIVOT_D, 0);

        upperPivotController.setP(ArmConstants.kUPPER_PIVOT_P, 0);
        upperPivotController.setI(ArmConstants.kUPPER_PIVOT_I, 0);
        upperPivotController.setD(ArmConstants.kUPPER_PIVOT_D, 0);

        upperPivotController.setPositionPIDWrappingEnabled(false);

        // Config sensors
        //elevatorLimitSwitch.enableLimitSwitch(true);
        elevatorExtEncoder.setPosition(0.0);

        lowerPivotEncoder.setInverted(false);
        lowerPivotEncoder.setZeroOffset(ArmConstants.kLOWER_ANGLE_OFFSET);

        upperPivotEncoder.setInverted(false);
        upperPivotEncoder.setZeroOffset(ArmConstants.kUPPER_ANGLE_OFFSET);

        lowerRightPivotController.setFeedbackDevice(lowerPivotEncoder);

        upperPivotRelEncoder.setPosition(upperPivotEncoder.getPosition() / ArmConstants.kUPPER_PIVOT_RATIO);
        upperPivotController.setFeedbackDevice(upperPivotRelEncoder);

        // Soft limits
        /*upperPivot.setSoftLimit(SoftLimitDirection.kForward, (float)(1.0 / ArmConstants.kUPPER_PIVOT_RATIO));
        upperPivot.setSoftLimit(SoftLimitDirection.kReverse, 0);

        upperPivot.enableSoftLimit(SoftLimitDirection.kForward, true);
        upperPivot.enableSoftLimit(SoftLimitDirection.kReverse, true);*/

        // Default
        setpoint = ArmPositions.kDEFAULT;
        setState(setpoint);
        
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
            new ArmToStateCommand(this, ArmPositions.kDEFAULT)
        );
    }

    /**
     * Sets the lower pivot's setpoint
     * @param angle degrees. 90 is down (unsafe), 180 is up
     */
    public void setLowerPivotAngle(double angle) {
        // Safety
        angle = Math.max(90.0, Math.min(angle, 190.0));

        lowerRightPivotController.setReference(angle / 360.0, ControlType.kPosition);

        setpoint.lowerAngle = angle;

        lowerPivotSetpointLog.append(angle);
    }

    /**
     * Sets the upper pivot's setpoint.
     * The lower pivot must be at a safe angle before the upper pivot will move.
     * @param angle degrees. 90 is downward, 180 is forward
     */
    public void setUpperPivotAngle(double angle) {
        // Safety
        angle = Math.max(5.0, Math.min(angle, 355.0));

        // Upper pivot motor set in periodic()
        setpoint.upperAngle = angle;

        upperPivotSetpointLog.append(angle);
    }

    /**
     * Sets the elevator's extension
     * @param rots rotations of the sprocket. 0 is inward, max is 2.72
     */
    public void setExtension(double rots) {
        // Safety
        rots = Math.max(0.0, Math.min(rots, 68.0 * ArmConstants.kELEVATOR_EXT_RATIO));

        elevatorExtController.setReference(rots / ArmConstants.kELEVATOR_EXT_RATIO , ControlType.kPosition, 0);

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
        // Check if upper pivot safe to move
        if (lowerPivotEncoder.getPosition() >= ArmConstants.kUPPER_PIVOT_MIN_ARM_ANGLE) {
            // Safe to spin
            upperPivotController.setReference(setpoint.upperAngle / 360.0 / ArmConstants.kUPPER_PIVOT_RATIO, ControlType.kPosition, 0);
            upperPivotSafeToMoveLog.append(true);
        } else {
            // Unsafe to spin
            upperPivotController.setReference(ArmPositions.kDEFAULT.upperAngle / 360.0 / ArmConstants.kUPPER_PIVOT_RATIO, ControlType.kPosition, 0);
            upperPivotSafeToMoveLog.append(false);
        }

        // Log
        lowerPivotPositionLog.append(lowerPivotEncoder.getPosition() * 360.0);

        upperPivotPositionLog.append(upperPivotEncoder.getPosition() * 360.0);
        
        elevatorExtPositionLog.append(elevatorExtEncoder.getPosition() * ArmConstants.kELEVATOR_EXT_RATIO);
		elevatorLimitSwitchLog.append(elevatorLimitSwitch.isPressed());
    }

    // TEST MODE //
    private GenericEntry testLowerPivot;
    private GenericEntry testUpperPivot;
    private GenericEntry testExt; 
    private GenericEntry testLowerPivotPValue;
    private GenericEntry testLowerPivotAbsEncOut;
    private GenericEntry testUpperPivotAbsEncOut;
    private GenericEntry testExtEncOut;

    /** Initializes shuffleboard values */
    public void initTestMode() {
        if (testLowerPivot != null) return; // Already initialized

        ShuffleboardTab tab = Shuffleboard.getTab("Arm Debug");
        
        testLowerPivot = tab.add("Lower Pivot SP", ArmPositions.kDEFAULT.lowerAngle)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();

        testUpperPivot = tab.add("Upper Pivot SP", ArmPositions.kDEFAULT.upperAngle)
            .withPosition(2, 0)
            .withSize(2, 1)
            .getEntry();

        testExt = tab.add("Ext SP", ArmPositions.kDEFAULT.ext)
            .withPosition(4, 0)
            .withSize(2, 1)
            .getEntry();

        testLowerPivotPValue = tab.add("Lower Pivot P", 0.0)
            .withPosition(6, 0)
            .withSize(2, 1)
            .getEntry();

        testLowerPivotAbsEncOut = tab.add("Lower Pivot Encoder", -1.0)
            .withSize(2, 1)
            .withPosition(0, 1)
            .getEntry();
        
        testUpperPivotAbsEncOut = tab.add("Upper Pivot Encoder", -1.0)
            .withSize(2, 1)
            .withPosition(2, 1)
            .getEntry();

        testExtEncOut = tab.add("Ext Encoder", -1.0)
            .withSize(2, 1)
            .withPosition(4, 1)
            .getEntry();
    }

    /** Runs test mode */
    public void execTestMode(boolean acceptOperatorInput) {
        if (acceptOperatorInput) {
            setUpperPivotAngle(testUpperPivot.getDouble(ArmPositions.kDEFAULT.upperAngle));
            setLowerPivotAngle(testLowerPivot.getDouble(ArmPositions.kDEFAULT.lowerAngle));
            setExtension(testExt.getDouble(ArmPositions.kDEFAULT.ext));
        }

        // Log values to shuffleboard
        testLowerPivotAbsEncOut.setDouble(lowerPivotEncoder.getPosition() * 360.0);
        testUpperPivotAbsEncOut.setDouble(upperPivotRelEncoder.getPosition() * 360.0 * ArmConstants.kUPPER_PIVOT_RATIO);
        testExtEncOut.setDouble(elevatorExtEncoder.getPosition() * ArmConstants.kELEVATOR_EXT_RATIO);

        // Set P value (disabled)
        /*
        double lowerPivotP = testLowerPivotPValue.getDouble(0.0);
        if (lowerRightPivotController.getP(0) != lowerPivotP) {
            System.out.printf("Lower pivot P set -> %f\n", lowerPivotP);
            lowerRightPivotController.setP(lowerPivotP, 0);
        }*/

        // Periodic (to populate log and move upper pivot if safe)
        periodic();
    }

    /** Stops test mode */
    public void endTestMode() {
        setState(ArmPositions.kDEFAULT);

        periodic();
    }
}
