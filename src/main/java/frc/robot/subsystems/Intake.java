package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake  {
    private CANSparkMax intake = new CANSparkMax(HardwareConstants.kINTAKE_CAN, MotorType.kBrushless);


    public Intake() {
        
        intake.restoreFactoryDefaults();
    }

    public void startIntake() {
        intake.set(kINTAKE_VELOCITY);
    }

    public void startOuttake() {
        intake.set(kINTAKE_VELOCITY * -1);
    }

    public void end() {
        intake.set(0);
    }
}
 {

}
