package frc.robot.commands;

import frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ClimberStageCommand extends SequentialCommandGroup{
    /**
     * sets the climber up or down
     * @param climber
     * @param climberSet true = up false = down
     */
    public ClimberStageCommand(Climber climber, boolean climberSet){
        super(
            //new WaitUntilCommand(() -> climberSet == true),
            new InstantCommand(() -> {
                if (climberSet) {climber.climberUp(); }
                else { climber.climberDown(); }
            }, climber),
            new WaitUntilCommand(() -> climber.climberAtSetpoint())
        );
    }
}



