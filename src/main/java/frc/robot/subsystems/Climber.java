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
public class Climber {
    //hardware
    private CANSparkMax climberLeft = new CANSparkMax(HardwareConstants.kCLIMBER_LEFT_CAN, MotorType.kBrushless);
    private CANSparkMax climberRight = new CANSparkMax(HardwareConstants.kCLIMBER_RIGHT_CAN, MotorType.kBrushless);

    private RelativeEncoder climberLeftEncoder = climberLeft.getEncoder();
    private RelativeEncoder climberRightEncoder = climberRight.getEncoder();

    private SparkPIDController climberLeftController;
    private SparkPIDController climberRightController;
        
    

    //init
    public Climber() {
        climberLeft.setIdleMode(IdleMode.kBrake);
        climberRight.setIdleMode(IdleMode.kBrake);
        climberLeft.setInverted(false);
        climberRight.setInverted(false);
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





    }
    /**Runs the intake forward */
    public void ClimberUp() {

        climberLeftController.setReference(ClimberConstants.kCLIMBER_POSITION, ControlType.kPosition, 0);
        climberRightController.setReference(ClimberConstants.kCLIMBER_POSITION, ControlType.kPosition, 0);



    }

    public void ClimberDown() {
        climberLeftController.setReference(ClimberConstants.kCLIMBER_POSITION, ControlType.kPosition, 0);
        climberRightController.setReference(ClimberConstants.kCLIMBER_POSITION, ControlType.kPosition, 0);

        
    }

    public void periodic() {

    }


}
