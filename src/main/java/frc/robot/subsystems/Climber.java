package frc.robot.subsystems;

import frc.robot.Constants.HardwareConstants;
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
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/LeftSetPoint", "rotations");
    private DoubleLogEntry climberRightSetPointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/RightSetPoint", "rotations");

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

        climberLeftController.setP(ClimberConstants.kCLIMBER_P, 0);
        climberLeftController.setI(ClimberConstants.kCLIMBER_I, 0);
        climberLeftController.setD(ClimberConstants.kCLIMBER_D, 0);

        climberRightController.setP(ClimberConstants.kCLIMBER_P, 0);
        climberRightController.setI(ClimberConstants.kCLIMBER_I, 0);
        climberRightController.setD(ClimberConstants.kCLIMBER_D, 0);

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

    /**Is the climber at its setpoint? */
    public boolean climberAtSetpoint(){
        return Math.abs(climberLeftEncoder.getPosition() - lSetpoint) <= ClimberConstants.kCLIMBER_TOLERANCE && 
            Math.abs(climberRightEncoder.getPosition() - rSetpoint) <= ClimberConstants.kCLIMBER_TOLERANCE;
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
