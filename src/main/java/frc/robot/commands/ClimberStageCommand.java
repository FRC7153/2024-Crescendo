package frc.robot.commands;

import frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ClimberStageCommand extends InstantCommand {
    /**
     * sets the climber up or down
     * @param climber
     * @param climberSet rots
     */
    public ClimberStageCommand(Climber climber, double climberSet){
        super(() -> {
                climber.setClimberHeight(climberSet);
        }, climber);
    }
}



