package frc.robot.subsystems;

import frc.robot.Constants.HardwareConstants;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Subsystem;


import com.revrobotics.CANSparkMax;
import com.frc7153.diagnostics.DiagUtil;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
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

    private DoubleLogEntry climberLeftPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/LeftPosition", "rotations");
    private DoubleLogEntry climberRightPositionLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/RightPosition", "rotations");
    private DoubleLogEntry climberLeftSetPointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/LeftSetPoint", "rotations");
    private DoubleLogEntry climberRightSetPointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Climber/RightSetPoint", "rotations");
    //init
    public Climber() {
        climberLeft.setIdleMode(IdleMode.kBrake);
        climberRight.setIdleMode(IdleMode.kBrake);
        climberLeft.setInverted(false);//TBD
        climberRight.setInverted(false);//TBD
        climberLeft.setSmartCurrentLimit(ClimberConstants.kCLIMBER_CURRENT_LIMIT);
        climberRight.setSmartCurrentLimit(ClimberConstants.kCLIMBER_CURRENT_LIMIT);

        climberLeftController = climberLeft.getPIDController();
        climberRightController = climberRight.getPIDController();

        climberLeftController.setP(ClimberConstants.kCLIMBER_P, 0);
        climberLeftController.setI(ClimberConstants.kCLIMBER_I, 0);
        climberLeftController.setD(ClimberConstants.kCLIMBER_D, 0);

        climberRightController.setP(ClimberConstants.kCLIMBER_P, 0);
        climberRightController.setP(ClimberConstants.kCLIMBER_I, 0);
        climberRightController.setP(ClimberConstants.kCLIMBER_D, 0);

        //config logging
        DiagUtil.addDevice(climberLeft);
        DiagUtil.addDevice(climberRight);
    
        climberLeftSetPointLog.append(0.0);
        climberRightSetPointLog.append(0.0);

        register();
    }

    /**Puts the climber up */
    public void climberUp() {
        climberLeftController.setReference(ClimberConstants.kCLIMBER_POSITION, ControlType.kPosition, 0);
        climberRightController.setReference(ClimberConstants.kCLIMBER_POSITION, ControlType.kPosition, 0);

        climberLeftSetPointLog.append(ClimberConstants.kCLIMBER_POSITION);
        climberRightSetPointLog.append(ClimberConstants.kCLIMBER_POSITION);
    }

    /**Puts the climber down */
    public void climberDown() {
        climberLeftController.setReference(0.0, ControlType.kPosition, 0);
        climberRightController.setReference(0.0, ControlType.kPosition, 0);

        climberRightSetPointLog.append(0);//to be changed
        climberLeftSetPointLog.append(0);//to be changed
    }

    @Override
    public void periodic() {
        climberRightPositionLog.append(climberRightEncoder.getPosition());
        climberLeftPositionLog.append(climberLeftEncoder.getPosition());
    }
}
