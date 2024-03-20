package frc.robot.auto;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.IndexerRegripCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.util.Util;

public class AutoUtils {
  /**
   * Create a new command to follow the path {@code pathName}.path
   * @param base
   * @param pathName
   * @param setPosition If the base's estimator should be reset here (only if this is the first path)
   */
  public static Command createFollowPathCommand(SwerveBase base, String pathName, boolean setPosition) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    FollowPathHolonomic pathCommand = new FollowPathHolonomic(
      path,
      () -> base.getPosition(true), // Robot pose supplier
      base::getRobotRelativeChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      base::driveChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      AutoConstants.kAUTO_PATH_FOLLOWER_CONFIG,
      Util::isRedAlliance,
      base
    );

    if (setPosition) {
      // Reset position first
      return new SequentialCommandGroup(
        new InstantCommand(() -> base.resetPosition(
          // Invert if red:
          (Util.isRedAlliance()) ? FieldConstants.INVERT_ALLIANCE(path.getPreviewStartingHolonomicPose()) : path.getPreviewStartingHolonomicPose()
        )),
        pathCommand
      );
    } else {
      // Don't reset position
      return pathCommand;
    }
  }

  public static Command rearSpeakerSubwooferShotCommand(Arm arm, Shooter shooter, Indexer indexer) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> arm.setState(ArmPositions.kSUBWOOFER_SPEAKER_REAR), arm),
      new InstantCommand(() -> shooter.setShootVelocity(3500.0), shooter),
      new WaitCommand(1.25),
      new InstantCommand(() -> indexer.setIndexerVelocity(700.0), indexer),
      new WaitCommand(0.5),
      new InstantCommand(indexer::stop, indexer)
    );
  }

  public static Command finishIntakingCommand(Indexer indexer, Intake intake) {
    return new ParallelCommandGroup(
      new InstantCommand(intake::end),
      new IndexerRegripCommand(indexer)
    );
  }
}
