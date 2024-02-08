package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

public class ArmShooterCommand extends ConditionalCommand {

    public ArmShooterCommand(Shooter shooter, boolean overrideSensor) {
        super(
            new SequentialCommandGroup(null), 
            new InstantCommand(), 
            () -> {return overrideSensor || StateController.getState().equals(NoteState.LOADED)}
        );
    }
    
}
