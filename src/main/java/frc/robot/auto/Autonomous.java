package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.SwerveBase;

public class Autonomous {
    // Shuffleboard entry
    private SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();
    private Command loadedAutoCommand = new PrintCommand("No auto loaded");

    // Init
    public Autonomous(SwerveBase base, Arm arm, Shooter shooter, Indexer indexer) {
        // Load selected auto
        chooser.onChange(this::autoOptionChanged);

        // No-Op
        chooser.setDefaultOption("No-Op", () -> { return new PrintCommand("No-Op Auto running..."); });

        // Time based autos
        chooser.addOption("Time-based Drive", () -> new TimeBasedAutos(base, arm, shooter, indexer, false));
        chooser.addOption("TIme-based Speaker/Drive", () -> new TimeBasedAutos(base, arm, shooter, indexer, true));
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
