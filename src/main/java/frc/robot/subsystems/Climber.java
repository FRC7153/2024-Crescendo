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



public class Climber {
    
    private CANSparkMax climberLeft = new CANSparkMax(HardwareConstants.kCLIMBER_LEFT_CAN, MotorType.kBrushless);
    private CANSparkMax climberRight = new CANSparkMax(HardwareConstants.kCLIMBER_RIGHT_CAN, MotorType.kBrushless);

    public Climber() {

    }

    public void ClimberUp() {

    }

    public void ClimberDown() {

    }

    public void periodic() {

    }


}
