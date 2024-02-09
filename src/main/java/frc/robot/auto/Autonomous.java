package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Shooter;

public class Autonomous {
    // Shuffleboard entry
    private SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();

    // Init
    public Autonomous(Shooter shooter) {
        // No-Op
        chooser.setDefaultOption("No-Op", () -> { return new PrintCommand("No-Op Auto"); });

        // Sys-Id Tuning
        chooser.addOption("SYSID - Shooter QS+", () -> shooter.sysIdRoutine.quasistatic(Direction.kForward));
        chooser.addOption("SYSID - Shooter QS-", () -> shooter.sysIdRoutine.quasistatic(Direction.kReverse));
        chooser.addOption("SYSID - Shooter D+", () -> shooter.sysIdRoutine.dynamic(Direction.kForward));
        chooser.addOption("SYSID - Shooter D-", () -> shooter.sysIdRoutine.dynamic(Direction.kReverse));

        // Add to Shuffleboard
        Shuffleboard.getTab("Drive").add("Autonomous", chooser);
    }

    // Get selected
    public Command getSelected() {
        return chooser.getSelected().get();
    }
}
