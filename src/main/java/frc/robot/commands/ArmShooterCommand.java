package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

public class ArmShooterCommand extends ConditionalCommand {

    public ArmShooterCommand(Shooter shooter, boolean overrideSensor) {
        super(
            new SequentialCommandGroup(
                // Wait until NOTE LOADED
                new WaitUntilCommand(() -> StateController.getState().equals(NoteState.LOADED)),
                // TODO move arm
                // Set setpoints
                new InstantCommand(() -> shooter.setShootVelocity(25), shooter),
                // Wait until setpoints reached
                new WaitUntilCommand(shooter::atShootSetpoint)
            ),
            new PrintCommand("OPERATOR tried to ARM shooter without NOTE loaded!"), 
            () -> { return overrideSensor || !StateController.getState().equals(NoteState.EMPTY); }
        );
    }
}
