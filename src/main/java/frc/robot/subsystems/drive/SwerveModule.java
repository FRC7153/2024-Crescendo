package frc.robot.subsystems.drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frc7153.diagnostics.DiagUtil;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import static com.revrobotics.CANSparkMax.ControlType;
import static com.revrobotics.CANSparkMax.IdleMode;
import static com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * A single swerve module, with a Falcon500 for drive, NEO for steer, and CANCoder
 * for heading.
 */
public class SwerveModule {
    /**
     * CRITICAL TODO: coupling ratio math needs to be optimized, and telemetry-reporting methods
     * need to respect the coupling ratio. Difference in steer needs to be periodically integrated
     * to drive motor position with coupling ratio.
     */

    // Hardware
    private TalonFX driveMotor;
    private CANSparkMax steerMotor; 
    private CANcoder steerCANCoder;

    // Control
    private MotionMagicVelocityVoltage driveControl;
    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private SparkPIDController steerPIDControl;
    private RelativeEncoder steerRelEncoder;
    private StatusSignal<Double> steerCANCoderPos;
    private StatusSignal<Double> steerCANCoderVelocity;

    // State
    private SwerveModuleState setpoint = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

    /**
     * Creates a new Swerve Module
     * @param driveCan (on the Canivore's bus)
     * @param steerCan
     * @param cancoderCan
     * @param steerZeroPos Added to angle of wheel(degrees, CCW positive)
     */
    public SwerveModule(
        int driveCan,
        int steerCan,
        int cancoderCan,
        double steerZeroPos
    ) {
        // Create and config DRIVE MOTOR
        driveMotor = new TalonFX(driveCan, HardwareConstants.kCANIVORE_BUS);
        driveMotor.setInverted(true); // Positive = CW rotation of motor, when observed from the top

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.Voltage.PeakForwardVoltage = SwerveModuleConstants.kDRIVE_PEAK_VOLTAGE;
        driveConfig.Voltage.PeakReverseVoltage = -SwerveModuleConstants.kDRIVE_PEAK_VOLTAGE;
        
        driveConfig.CurrentLimits.StatorCurrentLimit = SwerveModuleConstants.kDRIVE_STATOR_CURRENT_LIMIT;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = SwerveModuleConstants.kDRIVE_SUPPLY_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        driveConfig.Slot0.kP = SwerveModuleConstants.kDRIVE_P;
        driveConfig.Slot0.kI = SwerveModuleConstants.kDRIVE_I;
        driveConfig.Slot0.kD = SwerveModuleConstants.kDRIVE_D;
        driveConfig.Slot0.kS = SwerveModuleConstants.kDRIVE_S;
        driveConfig.Slot0.kV = SwerveModuleConstants.kDRIVE_V;
        driveConfig.Slot0.kA = SwerveModuleConstants.kDRIVE_A;
        
        driveConfig.MotionMagic.MotionMagicCruiseVelocity = SwerveModuleConstants.kDRIVE_MAX_VELO;
        driveConfig.MotionMagic.MotionMagicAcceleration = SwerveModuleConstants.kDRIVE_MAX_ACCEL;

        driveMotor.getConfigurator().apply(driveConfig);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);

        driveControl = new MotionMagicVelocityVoltage(0.0);
        driveControl.Slot = 0;
        driveMotor.setControl(driveControl);
        
        drivePosition = driveMotor.getRotorPosition();
        driveVelocity = driveMotor.getRotorVelocity();

        driveMotor.setPosition(0.0); // Ensure start at 0

        // Create and config STEER CANCODER
        steerCANCoder = new CANcoder(cancoderCan, HardwareConstants.kCANIVORE_BUS);

        CANcoderConfiguration steerCANCoderConfigs = new CANcoderConfiguration();
        steerCANCoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        steerCANCoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        steerCANCoderConfigs.MagnetSensor.MagnetOffset = (steerZeroPos / 360.0);
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

        DiagUtil.addDevice(drivePosition, driveMotor);
        DiagUtil.addDevice(driveVelocity, driveMotor);
        DiagUtil.addDevice(steerCANCoderPos, steerCANCoder);
        DiagUtil.addDevice(steerCANCoderVelocity, steerCANCoder);
    }

    /**
     * Gets the steer angle from the CANCoder, falling back to NEO built-in
     * encoder if necessary
     * @return Rotations (CCW+)
     */
    private double getSteerAnglePosRots() {
        steerCANCoderPos.refresh();
        if (steerCANCoderPos.getTimestamp().getLatency() < 2.5) { // CANCoder good
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
        if (steerCANCoderVelocity.getTimestamp().getLatency() < 2.5) { // CANCoder good
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
        DiagUtil.evaluateResponse(
            steerPIDControl.setReference(angle, ControlType.kPosition, 0),
        steerMotor);
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

        // Compensate for steer velocity
        velocity *= SwerveModuleConstants.kDRIVE_STAGE_2_RATIO; // Velocity of double gear
        /* Drive wheel forward is defined as clockwise rotation of the double gear,
         * when observed from the top */
        velocity -= getSteerAngleVelocity(); // Subtract speed of pulley

        // Set drive motor
        velocity *= SwerveModuleConstants.kDRIVE_STAGE_1_RATIO; // Velocity of motor
        DiagUtil.evaluateResponse(driveMotor.setControl(driveControl.withVelocity(velocity)), driveMotor);
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
        driveVelocity.refresh();
        
        return new SwerveModuleState(
            driveVelocity.getValue() / SwerveModuleConstants.kDRIVE_RATIO * SwerveModuleConstants.kWHEEL_CIRCUMFERENCE,
            Rotation2d.fromRotations(getSteerAnglePosRots())
        );
    }

    /**
     * Returns the current position of the swerve module
     * @return
     */
    public SwerveModulePosition getPosition() {
        drivePosition.refresh();

        return new SwerveModulePosition(
            drivePosition.getValue() / SwerveModuleConstants.kDRIVE_RATIO * SwerveModuleConstants.kWHEEL_CIRCUMFERENCE,
            Rotation2d.fromRotations(getSteerAnglePosRots())
        );
    }
}
