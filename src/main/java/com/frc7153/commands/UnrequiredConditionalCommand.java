package com.frc7153.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Similar to WPILib's ConditionalCommand, but does NOT require the requirements of its onTrue and
 * onFalse commands. Useful for checking if certain conditions are true BEFORE suspending other
 * commands.
 */
public class UnrequiredConditionalCommand extends Command {
  private Command onTrue, onFalse, selected;
  private BooleanSupplier condition;

  public UnrequiredConditionalCommand(Command onTrue, Command onFalse, BooleanSupplier condition) {
    this.onTrue = onTrue;
    this.onFalse = onFalse;
    this.condition = condition;
  }

  @Override
  public void initialize() {
    if (condition.getAsBoolean()) selected = onTrue;
    else selected = onFalse;

    if (selected != null) selected.schedule();
  }

  @Override
  public void end(boolean terminated) {
    if (selected != null && !selected.isFinished()) selected.cancel();
  }

  @Override
  public boolean isFinished() {
    if (selected != null) return selected.isFinished();
    else return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return ((onTrue == null || onTrue.runsWhenDisabled()) &&
      (onFalse == null || onFalse.runsWhenDisabled()));
  }
}
