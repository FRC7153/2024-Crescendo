package frc.robot.subsystems;

import frc.robot.Constants.HardwareConstants;
import frc.robot.util.Util;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;


import com.revrobotics.CANSparkMax;
import com.frc7153.diagnostics.DiagUtil;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.ClimberConstants;


/**Extends the climber */
public class Climber implements Subsystem {
    //hardware
    private CANSparkMax climberLeft = new CANSparkMax(HardwareConstants.kCLIMBER_LEFT_CAN, MotorType.kBrushless);
    private CANSparkMax climberRight = new CANSparkMax(HardwareConstants.kCLIMBER_RIGHT_CAN, MotorType.kBrushless);

    private RelativeEncoder climberLeftEncoder = climberLeft.getEncoder();
    private RelativeEncoder climberRightEncoder = climberRight.getEncoder();

    private SparkPIDController climberLeftController;
    private SparkPIDController climberRightController;

    // Output
    private GenericPublisher leftClimberPosOut, rightClimberPosOut;

    private DoubleLogEntry climberLeftPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/LeftPosition", "rotations");
    private DoubleLogEntry climberRightPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/RightPosition", "rotations");
    private DoubleLogEntry climberLeftSetPointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/LeftPosSetPoint", "rotations");
    private DoubleLogEntry climberRightSetPointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/RightPosSetPoint", "rotations");
    private DoubleLogEntry climberLeftVeloSetpointLog = 
            new DoubleLogEntry(DataLogManager.getLog(), "Climber/LeftVeloSetpoint", "rotations/sec");
    private DoubleLogEntry climberRightVeloSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/RightVeloSetpoint", "rotations/sec");

    private double lSetpoint, rSetpoint;

    //init
    public Climber() {
        // Clear some problematic configs
        climberLeft.restoreFactoryDefaults();
        climberRight.restoreFactoryDefaults();

        climberLeft.setIdleMode(IdleMode.kBrake);
        climberRight.setIdleMode(IdleMode.kBrake);
        climberLeft.setInverted(true);
        climberRight.setInverted(false);
        climberLeft.setSmartCurrentLimit(ClimberConstants.kCLIMBER_CURRENT_LIMIT);
        climberRight.setSmartCurrentLimit(ClimberConstants.kCLIMBER_CURRENT_LIMIT);

        climberLeftController = climberLeft.getPIDController();
        climberRightController = climberRight.getPIDController();

        // Set PID
        SparkPIDController[] controllers = {climberLeftController, climberRightController};

        for (SparkPIDController climberController : controllers) {
            // Position control
            climberController.setP(ClimberConstants.kCLIMBER_P, 0);
            climberController.setI(ClimberConstants.kCLIMBER_I, 0);
            climberController.setD(ClimberConstants.kCLIMBER_D, 0);

            // Velocity control
            climberController.setP(ClimberConstants.kCLIMBER_P, 1);
            climberController.setI(ClimberConstants.kCLIMBER_I, 1);
            climberController.setD(ClimberConstants.kCLIMBER_D, 1);
        }

        // config output
        if (BuildConstants.kOUTPUT_ALL_TELEMETRY) {
            ShuffleboardTab tab = Shuffleboard.getTab("Climber Telemetry");

            leftClimberPosOut = tab.add("Left Pos (rots)", -1.0)
                .getEntry().getTopic().genericPublish("double");
            
            rightClimberPosOut = tab.add("Right Pos (rots)", -1.0)
                .getEntry().getTopic().genericPublish("double");
        }

        //config logging
        DiagUtil.addDevice(climberLeft);
        DiagUtil.addDevice(climberRight);
    
        setClimberHeight(0.0);

        register();

        // Reduce CAN usage
        Util.disableExternalEncoderFrames(climberLeft);
        Util.disableExternalEncoderFrames(climberRight);
    }

    /**
     * Sets the climber height
     * @param height rots of motor (0 = down, 72 is max)
     */
    public void setClimberHeight(double leftHeight, double rightHeight) {
        // Safety
        leftHeight = Math.max(0.0, Math.min(leftHeight, 72.0));
        rightHeight = Math.max(0.0, Math.min(rightHeight, 72.0));

        // Set
        climberLeftController.setReference(leftHeight, ControlType.kPosition, 0);
        climberRightController.setReference(rightHeight, ControlType.kPosition, 0);

        climberLeftSetPointLog.append(leftHeight);
        climberRightSetPointLog.append(rightHeight);

        climberLeftVeloSetpointLog.append(0.0);
        climberRightVeloSetpointLog.append(0.0);

        lSetpoint = leftHeight;
        rSetpoint = rightHeight;
    }

    /**
     * Sets the climber height
     * @param height rots (0 = down, 72 is max)
     */
    public void setClimberHeight(double height) {
        setClimberHeight(height, height);
    }

    /**
     * Set the velocity, rotations/sec of sprocket
     * @param left
     * @param right
     */
    public void setClimberVelocity(double left, double right) {
        // Safety
        left = MathUtil.clamp(left, -1.0, 1.0);
        right = MathUtil.clamp(right, -1.0, 1.0);

        // Gearing
        left /= ClimberConstants.kCLIMBER_RATIO;
        right /= ClimberConstants.kCLIMBER_RATIO;

        // Set
        climberLeftController.setReference(left, ControlType.kVelocity, 1);
        climberRightController.setReference(right, ControlType.kVelocity, 1);

        climberLeftVeloSetpointLog.append(left);
        climberRightVeloSetpointLog.append(right);

        climberLeftSetPointLog.append(0.0);
        climberRightSetPointLog.append(0.0);
    }

    /** Is either climber at the bottom? */
    public boolean isClimberFloored(){
        return climberLeftEncoder.getPosition() < 0.2 || climberRightEncoder.getPosition() < 0.2;
    }

    @Override
    public void periodic() {
        climberRightPositionLog.append(climberRightEncoder.getPosition());
        climberLeftPositionLog.append(climberLeftEncoder.getPosition());

        // Log
        if (BuildConstants.kOUTPUT_ALL_TELEMETRY) {
            leftClimberPosOut.setDouble(climberLeftEncoder.getPosition());
            rightClimberPosOut.setDouble(climberRightEncoder.getPosition());
        }
    }
}
