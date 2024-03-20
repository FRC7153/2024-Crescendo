package frc.robot.auto;

import java.util.function.Supplier;

import com.frc7153.commands.NeverEndingCommand;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.LoadShooterGroundCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveBase;

public class Autonomous {
    // Shuffleboard entry
    private SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();
    private Command loadedAutoCommand = new PrintCommand("No auto loaded! The init pos was not set!");

    // Subsystems
    private SwerveBase base;
    private Intake intake;
    private Arm arm;
    private Shooter shooter;
    private Indexer indexer;

    /** Init auto.
     * It is expected that the robot will ALWAYS start facing FORWARDS! (because of gyro)
     */
    public Autonomous(SwerveBase base, Intake intake, Arm arm, Shooter shooter, Indexer indexer) {
        // Save refs to subsystems
        this.base = base;
        this.intake = intake;
        this.arm = arm;
        this.shooter = shooter;
        this.indexer = indexer;

        // Config auto commands
        NamedCommands.registerCommand("BeginIntake", 
            new LoadShooterGroundCommand(arm, shooter, intake, indexer).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        /*NamedCommands.registerCommand("IntakeBegun", new ParallelCommandGroup(
            new WaitCommand(4.0),
            new InstantCommand(intake::end, intake),
            new IndexerRegripCommand(indexer)
        ));*/

        // Load selected auto
        chooser.onChange(this::autoOptionChanged);

        // No-Op
        chooser.setDefaultOption("No-Op", () -> { return new NoOpAuto(base); });

        // Time based autos
        //chooser.addOption("Time-based Drive", () -> new TimeBasedAutos(base, arm, shooter, indexer, false));
        //chooser.addOption("Time-based Speaker/Drive", () -> new TimeBasedAutos(base, arm, shooter, indexer, true));

        // Simple trajectory autos
        chooser.addOption("Simple forward", this::simpleForward);

        // Center Subwoofer Autos
        chooser.addOption("Center Subwoofer Double Note", this::buildDoubleNoteFromSpeakerCenter);
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
    
    /** Drive forward straight */
    private Command simpleForward() {
        return AutoUtils.createFollowPathCommand(base, "StraightLine", true);
    }

    /** Double note from speaker center */
    private Command buildDoubleNoteFromSpeakerCenter() {
        return new SequentialCommandGroup(
            AutoUtils.rearSpeakerSubwooferShotCommand(arm, shooter, indexer),
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new LoadShooterGroundCommand(arm, shooter, intake, indexer),
                    new NeverEndingCommand() // If the above command finishes, stay here until path done
                ),
                AutoUtils.createFollowPathCommand(base, "SubwooferFrontToMiddleNote", true)
            ),
            new WaitCommand(2.0),
            AutoUtils.finishIntakingCommand(indexer, intake),
            AutoUtils.createFollowPathCommand(base, "CenterNoteToSubwooferFront", false),
            AutoUtils.rearSpeakerSubwooferShotCommand(arm, shooter, indexer)
        );
    }

    /**
     * 45 Subwoofer 2 Note
     */

     private Command build45Subwoofer2Note(){
        return new SequentialCommandGroup(
            AutoUtils.rearSpeakerSubwooferShotCommand(arm, shooter, indexer), // TODO: With Shooter Regression for that specific angle
            new ParallelRaceGroup(
                new SequentialCommandGroup(
                    new LoadShooterGroundCommand(arm, shooter, intake, indexer),
                    new NeverEndingCommand()
                ), 
                AutoUtils.createFollowPathCommand(base, "45Subwoofer2Note", false),
            ), 
            new WaitCommand(0.7),
            new Inta
        )
     }
}
