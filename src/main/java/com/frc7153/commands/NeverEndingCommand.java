package com.frc7153.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that never ends
 */
public class NeverEndingCommand extends Command {
  @Override
  public boolean isFinished() { return false; }
}
