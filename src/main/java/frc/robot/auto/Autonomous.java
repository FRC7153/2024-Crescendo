package frc.robot.auto;

import java.util.function.Supplier;

import com.frc7153.commands.NeverEndingCommand;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ArmPositions;
import frc.robot.commands.ArmToStateCommand;
import frc.robot.commands.LoadShooterGroundCommand;
import frc.robot.commands.ShootCommand;
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
        chooser.addOption("Simple forward", this::buildSimpleForward);

        // Shoot and stay
        chooser.addOption("Single rear shot (left 45)", () -> buildShootDontMove(45.0));
        chooser.addOption("Single rear shot (center)", () -> buildShootDontMove(0.0));
        chooser.addOption("Single rear shot (right 45)", () -> buildShootDontMove(360.0 - 45.0));

        //Amp 1 Note
        chooser.addOption("Single Note Amp Shot (blue alliance)", () -> buildPerpendicularAmp1Note(270.0));
        chooser.addOption("Single Note Amp Shot (red alliance)", () -> buildPerpendicularAmp1Note(90.0));

        // Center Subwoofer Autos
        chooser.addOption("Center Subwoofer Double Note", this::buildDoubleNoteFromSpeakerCenter);
        chooser.addOption("Left 45 Subwoofer Double Note", this::build45Subwoofer2Note);
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
    private Command buildSimpleForward() {
        return AutoUtils.createFollowPathCommand(base, "StraightLine", true)
            .withName("Simple-forward");
    }

    /** Shoot and don't move */
    private Command buildShootDontMove(double startAngle) {
        return new SequentialCommandGroup(
            new WaitCommand(2.69), // nice
            new InstantCommand(() -> base.resetPosition(AutoUtils.defaultInitPos())),
            new InstantCommand(() -> base.setYawAngle(startAngle)),
            AutoUtils.rearSpeakerSubwooferShotCommand(arm, shooter, indexer),
            new PrintCommand("Shot, waiting...")
        ).withName(String.format("Shoot-do-not-move-%f", startAngle));
    }

    /**
     * Amp 90 or 270 Degrees 
     */
    private Command buildPerpendicularAmp1Note(double startAngle){
        return new SequentialCommandGroup(
            new InstantCommand(() -> base.resetPosition(AutoUtils.defaultInitPos())),
            new InstantCommand(() -> base.setYawAngle(startAngle)),
            new ParallelRaceGroup(
                new ArmToStateCommand(arm, ArmPositions.kREAR_AMP),
                new SequentialCommandGroup(
                    new WaitCommand(3.0),
                    new ShootCommand(indexer, false, () -> false)
                ),
                new WaitCommand(6.0)
            ),
            new ArmToStateCommand(arm, ArmPositions.kDEFAULT)
        ).withName(String.format("Single-shot-amp-%f", startAngle));
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
        ).withName("Double-note-from-speaker-center");
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
                AutoUtils.createFollowPathCommand(base, "45Subwoofer2Note", false)
            ), 
            AutoUtils.finishIntakingCommand(indexer, intake),
            AutoUtils.rearSpeakerSubwooferShotCommand(arm, shooter, indexer)
            //build45Subwoofer2Note() // TODO test
        );
     } 
}
