package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frc7153.diagnostics.DiagUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import static com.revrobotics.CANSparkMax.ControlType;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * A single swerve module, with a NEO for drive, NEO for steer, and CANCoder
 * for heading.
 */
public class SwerveModule {
    // Hardware
    private CANSparkMax driveMotor;
    private CANSparkMax steerMotor; 
    private CANcoder steerCANCoder;

    // Control
    private SparkPIDController drivePIDController;
    private RelativeEncoder driveEncoder;
    private SimpleMotorFeedforward driveFF;
    private SparkPIDController steerPIDControl;
    private RelativeEncoder steerRelEncoder;
    private StatusSignal<Double> steerCANCoderPos;
    private StatusSignal<Double> steerCANCoderVelocity;

    // Coupling ratio integration
    private double drivePosOffset = 0.0;
    private double lastSteerAngle = 0.0;

    // State
    private SwerveModuleState setpoint = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

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
        driveMotor.setInverted(true); // Positive = CW rotation of motor, when observed from the top
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.setSmartCurrentLimit(SwerveModuleConstants.kDRIVE_CURRENT_LIMIT);

        drivePIDController = driveMotor.getPIDController();

        drivePIDController.setP(SwerveModuleConstants.kDRIVE_P, 0);
        drivePIDController.setI(SwerveModuleConstants.kDRIVE_I, 0);
        drivePIDController.setD(SwerveModuleConstants.kDRIVE_D, 0);
        drivePIDController.setSmartMotionMaxAccel(SwerveModuleConstants.kDRIVE_MAX_ACCEL, 0);
        drivePIDController.setSmartMotionMaxVelocity(SwerveModuleConstants.kDRIVE_MAX_ACCEL, 0);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPosition(0.0);

        // Configure drive feed forward
        driveFF = new SimpleMotorFeedforward(
            SwerveModuleConstants.kDRIVE_S, 
            SwerveModuleConstants.kDRIVE_V, 
            SwerveModuleConstants.kDRIVE_A
        );

        // Create and config STEER CANCODER
        steerCANCoder = new CANcoder(cancoderCan, HardwareConstants.kCANIVORE_BUS);

        CANcoderConfiguration steerCANCoderConfigs = new CANcoderConfiguration();
        steerCANCoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        steerCANCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        steerCANCoderConfigs.MagnetSensor.MagnetOffset = steerZeroPos;
        steerCANCoder.getConfigurator().apply(steerCANCoderConfigs);

        steerCANCoderVelocity = steerCANCoder.getVelocity();

        steerCANCoderPos = steerCANCoder.getAbsolutePosition();
        steerCANCoderPos.waitForUpdate(1.5); // this is BLOCKING

        // Create and configure STEER MOTOR
        steerMotor = new CANSparkMax(steerCan, MotorType.kBrushless);
        steerMotor.setInverted(false); // Positive = CW rotation of motor, when observed from top
        steerRelEncoder = steerMotor.getEncoder();

        steerRelEncoder.setPosition(
            steerCANCoderPos.getValue() * SwerveModuleConstants.kSTEER_RATIO
        );

        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setSmartCurrentLimit(SwerveModuleConstants.kSTEER_CURRENT_LIMIT);

        steerPIDControl = steerMotor.getPIDController();
        steerPIDControl.setFeedbackDevice(steerRelEncoder);
        steerPIDControl.setP(SwerveModuleConstants.kSTEER_P, 0);
        steerPIDControl.setI(SwerveModuleConstants.kSTEER_I, 0);
        steerPIDControl.setD(SwerveModuleConstants.kSTEER_D, 0);
        //steerPIDControl.setOutputRange(-10.0, 10.0, 0);
        steerPIDControl.setPositionPIDWrappingMinInput(-0.5 * SwerveModuleConstants.kSTEER_RATIO);
        steerPIDControl.setPositionPIDWrappingMaxInput(0.5 * SwerveModuleConstants.kSTEER_RATIO);
        steerPIDControl.setPositionPIDWrappingEnabled(true);

        // Initiate diagnostics
        DiagUtil.addDevice(driveMotor);
        DiagUtil.addDevice(steerCANCoder);
        DiagUtil.addDevice(steerMotor);
    }

    /**
     * Gets the steer angle from the CANCoder, falling back to NEO built-in
     * encoder if necessary
     * @return Rotations (CCW+)
     */
    private double getSteerAnglePosRots() {
        steerCANCoderPos.refresh();
        if (steerCANCoderPos.getTimestamp().getLatency() < 2.0) { // CANCoder good
            return steerCANCoderPos.getValue();
        } else { // CANCoder bad
            return steerRelEncoder.getPosition() / SwerveModuleConstants.kSTEER_RATIO;
        }
    }

    /**
     * Gets the steer angle velocity from the CANCoder, falling back to NEO built-in
     * encoder if necessary
     * @return Rotations per second (CCW+)
     */
    private double getSteerAngleVelocity() {
        steerCANCoderVelocity.refresh();
        if (steerCANCoderVelocity.getTimestamp().getLatency() < 2.0) { // CANCoder good
            return steerCANCoderVelocity.getValue();
        } else { // CANCoder bad
            return steerRelEncoder.getVelocity() / SwerveModuleConstants.kSTEER_RATIO / 60.0;
        }
    }

    /**
     * Sets the heading of the steer motor
     * @param angle degrees (0 is forward, CCW positive)
     */
    public void setSteerAngle(double angle) {
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
        periodic(); // This will set the drive motor control
    }

    /**
     * Optimizes and sets the current velocity and angle
     * @param state
     */
    public void setModuleState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromRotations(getSteerAnglePosRots()));

        setSteerAngle(state.angle.getDegrees());
        setDriveWheelVelocity(state.speedMetersPerSecond);
    }

    /**
     * This should run periodically, updating the velocity wheel to account for steer
     * error and steer velocity.
     */
    public void periodic() {
        /* Multiply velocity setpoint by cosine of steer error to tune down the 
         * velocity when its pointing in the wrong direction */
        double steerErr = Math.abs(getSteerAnglePosRots() - setpoint.angle.getRotations()) * Math.PI * 2.0; // rads
        double velocity = setpoint.speedMetersPerSecond * Math.cos(steerErr);

        // Convert to wheel velocity (r/s)
        velocity /= SwerveModuleConstants.kWHEEL_CIRCUMFERENCE;

        // Compensate for coupling ratio
        double couplingEffect = getSteerAngleVelocity() * SwerveModuleConstants.kCOUPLING_RATIO;
        velocity -= couplingEffect;

        // Set drive motor
        drivePIDController.setReference(
            velocity, 
            ControlType.kSmartVelocity, 
            0, 
            driveFF.calculate(velocity, SwerveModuleConstants.kDRIVE_MAX_ACCEL)
        );

        // Compensate coupling ratio in odometry
        drivePosOffset += ((getSteerAnglePosRots() - lastSteerAngle) * SwerveModuleConstants.kCOUPLING_RATIO);
        lastSteerAngle = getSteerAnglePosRots(); // rots
    }

    /**
     * Returns the current swerve module setpoint
     * @return Most recently requested setpoint
     */
    public SwerveModuleState getSetpoint() { return setpoint; }

    /**
     * Returns the current state of the swerve module
     * @return  The current actual state
     */
    public SwerveModuleState getState() {
        double coupling = getSteerAngleVelocity() * SwerveModuleConstants.kCOUPLING_RATIO;    
        return new SwerveModuleState(
            ((driveEncoder.getVelocity() / SwerveModuleConstants.kDRIVE_RATIO) - coupling) * SwerveModuleConstants.kWHEEL_CIRCUMFERENCE,
            Rotation2d.fromRotations(getSteerAnglePosRots())
        );
    }

    /**
     * Returns the current position of the swerve module
     * @return
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            (driveEncoder.getPosition() / SwerveModuleConstants.kDRIVE_RATIO * SwerveModuleConstants.kWHEEL_CIRCUMFERENCE) - drivePosOffset,
            Rotation2d.fromRotations(getSteerAnglePosRots())
        );
    }
}
