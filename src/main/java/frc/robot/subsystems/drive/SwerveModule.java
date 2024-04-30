package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frc7153.controllers.SafeCANCoder;
import com.frc7153.diagnostics.DiagUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import static com.revrobotics.CANSparkMax.ControlType;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.util.Util;

/**
 * A single swerve module, with a NEO for drive, NEO for steer, and CANCoder
 * for heading.
 */
public class SwerveModule {
    // Hardware
    private CANSparkMax driveMotor;
    private CANSparkMax steerMotor; 
    private SafeCANCoder steerCANCoder;

    // Control
    private SparkPIDController drivePIDController;
    private RelativeEncoder driveEncoder;
    private SparkPIDController steerPIDControl;
    private RelativeEncoder steerRelEncoder;

    // State
    private SwerveModuleState setpoint = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

    // Output
    private DoubleEntry tuneDriveVoltsOut;

    /**
     * Creates a new Swerve Module
     * @param driveCan (on the Canivore's bus)
     * @param steerCan
     * @param cancoderCan
     * @param steerZeroPos Added to angle of wheel (rotations, CCW positive)
     */
    public SwerveModule(
        int driveCan,
        int steerCan,
        int cancoderCan,
        double steerZeroPos
    ) {
        // Create and config DRIVE MOTOR
        driveMotor = new CANSparkMax(driveCan, MotorType.kBrushless);
        driveMotor.setInverted(false); // Positive = CW rotation of motor, when observed from the top
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.setSmartCurrentLimit(SwerveModuleConstants.kDRIVE_CURRENT_LIMIT);

        drivePIDController = driveMotor.getPIDController();

        drivePIDController.setP(SwerveModuleConstants.kDRIVE_P, 0);
        drivePIDController.setI(SwerveModuleConstants.kDRIVE_I, 0);
        drivePIDController.setD(SwerveModuleConstants.kDRIVE_D, 0);

        drivePIDController.setIAccum(0.0);
        drivePIDController.setIMaxAccum(6.0, 0);
        //drivePIDController.setSmartMotionMaxAccel(SwerveModuleConstants.kDRIVE_MAX_ACCEL, 0);
        //drivePIDController.setSmartMotionMaxVelocity(SwerveModuleConstants.kDRIVE_MAX_ACCEL, 0);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPosition(0.0);

        // Create and config STEER CANCODER
        steerCANCoder = new SafeCANCoder(cancoderCan, HardwareConstants.kCANIVORE_BUS);

        steerCANCoder.setRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
        steerCANCoder.setMagnetDirection(SensorDirectionValue.CounterClockwise_Positive);
        steerCANCoder.setMagnetOffset(-steerZeroPos);

        // Create and configure STEER MOTOR
        steerMotor = new CANSparkMax(steerCan, MotorType.kBrushless);
        steerMotor.setInverted(true); // Positive = CW rotation of motor, when observed from top
        steerRelEncoder = steerMotor.getEncoder();

        steerRelEncoder.setPosition(
            steerCANCoder.getAbsolutePosition() * SwerveModuleConstants.kSTEER_RATIO
        );

        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setSmartCurrentLimit(SwerveModuleConstants.kSTEER_CURRENT_LIMIT);

        steerPIDControl = steerMotor.getPIDController();
        steerPIDControl.setFeedbackDevice(steerRelEncoder);
        steerPIDControl.setP(SwerveModuleConstants.kSTEER_P, 0);
        steerPIDControl.setI(SwerveModuleConstants.kSTEER_I, 0);
        steerPIDControl.setD(SwerveModuleConstants.kSTEER_D, 0);
        steerPIDControl.setFF(SwerveModuleConstants.kSTEER_FF, 0);
        //steerPIDControl.setOutputRange(-10.0, 10.0, 0);
        steerPIDControl.setPositionPIDWrappingMinInput(-0.5 * SwerveModuleConstants.kSTEER_RATIO);
        steerPIDControl.setPositionPIDWrappingMaxInput(0.5 * SwerveModuleConstants.kSTEER_RATIO);
        steerPIDControl.setPositionPIDWrappingEnabled(true);

        // Log volts if tuning drive
        NetworkTable nt = NetworkTableInstance.getDefault().getTable("Swerve Drive Volts");

        tuneDriveVoltsOut = 
            nt.getDoubleTopic(String.format("Module %d drive volts", driveCan)).getEntry(-99.0);

        tuneDriveVoltsOut.set(-99.0);

        // Initiate diagnostics
        DiagUtil.addDevice(driveMotor);
        DiagUtil.addDevice(steerMotor);
        steerCANCoder.initDiagnostics();

        // Reduce CAN usage
        Util.disableExternalEncoderFrames(driveMotor);
        Util.disableExternalEncoderFrames(steerMotor);
    }

    /**
     * Sets the P, I, D, and FF constants of the velocity controller.
     */
    public void setConfigs(double p, double i, double d, double ff) {
        // Safety check
        if (!BuildConstants.kDRIVE_TUNE_MODE) {
            DriverStation.reportError("Not in drive tune mode!", false);
            return;
        }

        // Set values
        drivePIDController.setP(p, 0);
        drivePIDController.setI(i, 0);
        drivePIDController.setD(d, 0);
        drivePIDController.setFF(ff, 0);
    }

    /** Checks cancoder vs spark rel encoder. CALL WHEN DISABLED */
    public void doubleCheckSteerEncoderPositions() {
        double abs = steerCANCoder.getAbsolutePosition();
        double rel = steerRelEncoder.getPosition() / SwerveModuleConstants.kSTEER_RATIO;

        System.out.printf("Checking a swerve module! Heading is off by %f rots\n", abs - rel);

        steerRelEncoder.setPosition(
            abs * SwerveModuleConstants.kSTEER_RATIO
        );
    }

    /**
     * Sets the heading of the steer motor
     * @param angle degrees (0 is forward, CCW positive)
     */
    public void setSteerAngle(double angle) {
        angle = MathUtil.inputModulus(angle, -180.0, 180.0);
        setpoint.angle = Rotation2d.fromDegrees(angle);
        angle = (angle/360.0) * SwerveModuleConstants.kSTEER_RATIO;
        steerPIDControl.setReference(angle, ControlType.kPosition, 0);
    }

    /**
     * Sets the drive motor's velocity
     * @param velocity meters/second
     */
    public void setDriveWheelVelocity(double velocity) {
        setpoint.speedMetersPerSecond = velocity;
        //periodic(); // This will set the drive motor control

        // Immediate:
        velocity /= SwerveModuleConstants.kWHEEL_CIRCUMFERENCE;
        velocity *= SwerveModuleConstants.kDRIVE_RATIO;
        velocity *= 60.0;

        drivePIDController.setReference(
            velocity,
            ControlType.kVelocity,
            0
        );
    }

    /**
     * Optimizes and sets the current velocity and angle
     * @param state
     */
    public void setModuleState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromRotations(steerCANCoder.getAbsolutePosition()));

        setSteerAngle(state.angle.getDegrees());
        setDriveWheelVelocity(state.speedMetersPerSecond);
    }

    /**
     * Returns the current swerve module setpoint
     * @return Most recently requested setpoint
     */
    public SwerveModuleState getSetpoint() { return setpoint; }

    /**
     * Returns the current state of the swerve module, using the CANCoder for heading
     * @return  The current actual state
     */
    public SwerveModuleState getStateWithCanivore() {
        return new SwerveModuleState(
            ((driveEncoder.getVelocity() / SwerveModuleConstants.kDRIVE_RATIO) * SwerveModuleConstants.kWHEEL_CIRCUMFERENCE) / 60.0,
            Rotation2d.fromRotations(steerCANCoder.getAbsolutePosition())
        );
    }

    /**
     * Returns the current state of the swerve module, using the relative encoder for heading
     * @return  The current actual state
     */
    public SwerveModuleState getStateWithSparkEnc() {
        return new SwerveModuleState(
            ((driveEncoder.getVelocity() / SwerveModuleConstants.kDRIVE_RATIO) * SwerveModuleConstants.kWHEEL_CIRCUMFERENCE) / 60.0,
            Rotation2d.fromRotations(steerRelEncoder.getPosition() / SwerveModuleConstants.kSTEER_RATIO)
        );
    } 

    /**
     * Returns the current position of the swerve module
     * @return
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition() / SwerveModuleConstants.kDRIVE_RATIO * SwerveModuleConstants.kWHEEL_CIRCUMFERENCE,
            Rotation2d.fromRotations(steerCANCoder.getAbsolutePosition())
        );
    }

    /**
     * Outputs data to NT
     */
    public void output() {
        tuneDriveVoltsOut.set(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
    }
}
