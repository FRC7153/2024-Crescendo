package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Shooter;

public class Autonomous {
    // Shuffleboard entry
    private SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();

    // Init
    public Autonomous(Shooter shooter) {
        // No-Op
        chooser.setDefaultOption("No-Op", () -> { return new PrintCommand("No-Op Auto"); });

       /* TODO: it is expected that autos will specify what OBJECTIVE STATE they want the robot in, and
       whether a NOTE is preloaded */

        // Add to Shuffleboard
        Shuffleboard.getTab("Drive").add("Autonomous", chooser)
            .withPosition(3, 1);
    }

    // Get selected
    public Command getSelected() {
        return chooser.getSelected().get();
    }
}
