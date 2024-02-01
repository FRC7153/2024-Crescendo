package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HardwareConstants;



public class Arm {
    public CANSparkMax lowerPivot = new CANSparkMax(HardwareConstants.kLOWER_PIVOT_CAN, MotorType.kBrushless);
    public CANSparkMax upperPivot = new CANSparkMax(HardwareConstants.kUPPER_PIVOT_CAN, MotorType.kBrushless);
}
