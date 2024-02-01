package com.frc7153.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Command that only runs in teleop mode.
 * Used for subsystem's default commands that should only run in teleop.
 */
public class TeleopCommand extends ConditionalCommand {
    // No-op command

    /**
     * @param teleop This command will run if in teleop
     * @param notTeleop This command will run if NOT in teleop
     */
    public TeleopCommand(Command teleop, Command notTeleop) {
        super(teleop, notTeleop, DriverStation::isTeleop);
    }

    /**
     * @param teleop This command will run if in teleop
     */
    public TeleopCommand(Command teleop) {
        super(teleop, new InstantCommand(), DriverStation::isTeleop);
    }
}
