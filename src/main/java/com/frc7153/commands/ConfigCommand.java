package com.frc7153.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Instant command that has a title and can run when disabled.
 * This can be put on Shuffleboard to configure subsystems.
 */
public class ConfigCommand extends InstantCommand {
    private String name;
    
    /**
     * Creates a new ConfigCommand
     * @param func The function to run
     * @param name Name for this command
     * @param requirements The subsystems required by this command
     */
    public ConfigCommand(Runnable func, String name, Subsystem... requirements) {
        super(func, requirements);
        this.name = name;
    }

    /**
     * Creates a new ConfigCommand
     * @param func The function to run
     * @param name A name for this command
     */
    public ConfigCommand(Runnable func, String name) {
        super(func);
        this.name = name;
    }

    // Config
    @Override
    public String getName() { return name; }

    @Override
    public boolean runsWhenDisabled() { return true; }
}
