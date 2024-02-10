package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

public class ArmAmpCommand extends ConditionalCommand {
    public ArmAmpCommand(Intake intake, boolean overrideSensor) {
        super(
            new SequentialCommandGroup(

            ),
            new PrintCommand("The operator tried to ARM the intake with the Note Loaded"),
            () -> { return overrideSensor || !StateController.getState().equals(NoteState.LOADED); }
        );

        addRequirements(intake);
    }
}
