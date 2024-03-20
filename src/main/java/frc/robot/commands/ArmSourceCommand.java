package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmPositions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ArmSourceCommand extends ParallelRaceGroup {
  /**
   * Moves the arm to intake from the Source
   * @param arm
   * @param shooter
   * @param indexer
   * @param button the button pressed to run this command
   */
  public ArmSourceCommand(Arm arm, Shooter shooter, Indexer indexer, Trigger button) {
    super(
      // Move arm
      new ArmToStateCommand(arm, ArmPositions.kSOURCE_INTAKE_FRONT, null, null, 0, 0),
      // Move wheels
      new SequentialCommandGroup(
        new InstantCommand(() -> shooter.setShootVelocity(-5.0), shooter), // Reverse shooter
        new InstantCommand(() -> indexer.setIndexerVelocity(0.0), indexer), // Stop indexer
        new WaitUntilCommand(button.negate()), // Wait until released
        new InstantCommand(() -> shooter.setShootVelocity(0.0), shooter), // Stop shooter
        new IndexerRegripCommand(indexer) // Regrip indexer
      )
    );
  }
}
