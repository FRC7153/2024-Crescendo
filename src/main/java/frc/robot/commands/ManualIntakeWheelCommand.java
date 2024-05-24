package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer;

public class ManualIntakeWheelCommand extends Command {
  private Indexer indexer;

  public ManualIntakeWheelCommand(Indexer indexer) {
    this.indexer = indexer;

    addRequirements(indexer);
  }

  @Override
  public void initialize() {
    indexer.setIndexerVelocity(400.0);
  }

  @Override
  public void end(boolean terminated) {
    indexer.stop();
  }
}
