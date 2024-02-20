package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPositions;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LED;

public class ArmAmpCommand extends RepeatCommand {
    /**
     * Arms the robot to shoot into the AMP.
     * Requires a NOTE to be LOADED.
     */
    public ArmAmpCommand(Arm arm, LED led, Supplier<Pose2d> poseSupplier) {
        super(new SequentialCommandGroup(
            // Move arm
            new ConditionalCommand(
                // Is facing amp
                new InstantCommand(() -> arm.setState(ArmPositions.kFRONT_AMP), arm), 
                // Is not facing amp
                new InstantCommand(() -> arm.setState(ArmPositions.kREAR_AMP), arm), 
                // Checks direction of robot
                () -> Math.abs(90.0 - poseSupplier.get().getRotation().getDegrees()) < 90.0
            ),
            // Set LEDs
            new ConditionalCommand(
                // At setpoint
                new InstantCommand(() -> led.setPulse(LEDConstants.kGREEN), led),
                // Not at setpoint
                new InstantCommand(led::setAllianceStationColor, led),
                arm::atSetpoint
            )
        ));
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
