package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.StateController;
import frc.robot.util.StateController.NoteState;

public class SetStateCommand extends InstantCommand {
  public SetStateCommand(NoteState state) {
    super(() -> StateController.setNoteState(state));
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
