package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake implements Subsystem {
    private CANSparkMax intake = new CANSparkMax(HardwareConstants.kINTAKE_CAN, MotorType.kBrushless);
    private SparkPIDController intakeController = intake.getPIDController();

    private RelativeEncoder intakeEncoder = intake.getEncoder();

    public Intake() {
        intake.setIdleMode(IdleMode.kBrake);
        intake.setInverted(false);
        intake.setSmartCurrentLimit(IntakeConstants.kINTAKE_CURRENt_LIMIT);

        intakeController = intake.getPIDController();
        intakeController.setP(IntakeConstants.kINTAKE_P, 0);
        intakeController.setI(IntakeConstants.kINTAKE_I, 0);
        intakeController.setD(IntakeConstants.kINTAKE_D, 0);




        //intake.restoreFactoryDefaults();
    }

    public void enableIntake(boolean enabled) {
        if(enabled){
            intakeController.setReference(IntakeConstants.kINTAKE_SETPOiNT * IntakeConstants.kINTAKE_RATIO, ControlType.kVelocity, 0);
        } else {
            intakeController.setReference(0.0, ControlType.kVelocity,0);
        }
        
        //intake.set(kINTAKE_VELOCITY);
    }

    public void reverseIntake(boolean enabled) {
        if(enabled){
            intakeController.setReference(IntakeConstants.kINTAKE_SETPOiNT * IntakeConstants.kINTAKE_RATIO * -1, ControlType.kVelocity, 0);
        } else {
            intakeController.setReference(0.0, ControlType.kVelocity, 0);
        }

        //intake.set(kINTAKE_VELOCITY);
    }

    public void end() {
        intakeController.setReference(0.0, ControlType.kVelocity, 0);
    }


}
