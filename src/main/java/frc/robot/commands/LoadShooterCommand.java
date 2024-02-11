package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * Moves the shooter to intake a NOTE from the ground.
 */
public class LoadShooterCommand extends ParallelCommandGroup {
    public LoadShooterCommand(Arm arm, Shooter shooter, Intake intake) {
        
    }
}
