package frc.robot.auto;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.IndexerRegripCommand;
import frc.robot.commands.LoadShooterGroundCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveBase;
import frc.robot.util.Util;

public class Autonomous {
    // Shuffleboard entry
    private SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();
    private Command loadedAutoCommand = new PrintCommand("No auto loaded");

    /** Init auto.
     * It is expected that the robot will ALWAYS start facing FORWARDS! (because of gyro)
     */
    public Autonomous(SwerveBase base, Intake intake, Arm arm, Shooter shooter, Indexer indexer) {
        // Config auto
        AutoBuilder.configureHolonomic(
            () -> base.getPosition(true), 
            base::resetPosition,
            base::getRobotRelativeChassisSpeeds,
            base::driveChassisSpeeds,
            AutoConstants.kAUTO_PATH_FOLLOWER_CONFIG,
            Util::isRedAlliance,
            base
        );

        // Config auto commands
        NamedCommands.registerCommand("BeginIntake", 
            new LoadShooterGroundCommand(arm, shooter, intake, indexer).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        NamedCommands.registerCommand("EndIntake", new ParallelCommandGroup(
            new InstantCommand(intake::end, intake)
        ));

        NamedCommands.registerCommand("RegripNote", new IndexerRegripCommand(indexer));

        NamedCommands.registerCommand("SpeakerRearShoot", new SequentialCommandGroup(
            new InstantCommand(() -> arm.setState(ArmPositions.kSUBWOOFER_SPEAKER_REAR), arm),
            new InstantCommand(() -> shooter.setShootVelocity(58.0), shooter),
            new WaitCommand(1.5),
            new InstantCommand(() -> indexer.setIndexerVelocity(700.0), indexer),
            new WaitCommand(1.0),
            new InstantCommand(indexer::stop, indexer)
        ));

        // Load selected auto
        chooser.onChange(this::autoOptionChanged);

        // No-Op
        chooser.setDefaultOption("No-Op", () -> { return new PrintCommand("No-Op Auto running..."); });

        // Time based autos
        chooser.addOption("Time-based Drive", () -> new TimeBasedAutos(base, arm, shooter, indexer, false));
        chooser.addOption("Time-based Speaker/Drive", () -> new TimeBasedAutos(base, arm, shooter, indexer, true));

        // Simple trajectory autos
        chooser.addOption("Simple forward", () -> new PathPlannerAuto("DriveForward"));

        // Center Subwoofer Autos
        chooser.addOption("Center Subwoofer Double Note", () -> new PathPlannerAuto("SubwooferCenter2Notes"));
    }

    // On change
    private void autoOptionChanged(Supplier<Command> commandSupplier) {
        loadedAutoCommand = commandSupplier.get();
        System.out.printf("Loaded new auto: '%s'\n", loadedAutoCommand.getName());
    }

    /** Get the chooser, to add to Shuffleboard */
    public SendableChooser<?> getChooser() {
        return chooser;
    }

    /** Get selected (and preloaded) command */
    public Command getSelected() {
        return loadedAutoCommand;
    }
}
