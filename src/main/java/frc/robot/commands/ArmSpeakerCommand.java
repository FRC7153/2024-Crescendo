package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShootingRegressions;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

public class ArmSpeakerCommand extends Command {
    // This math used for trig later
    private static double speakerToPivotHeight = FieldConstants.kSPEAKER_HEIGHT - DriveConstants.kARM_LOWER_PIVOT_HEIGHT;
    private static double speakerToReversedPivotHeight = speakerToPivotHeight - Units.feetToMeters(1.5 * Math.sin(Units.degreesToRadians(15.0)));

    // Subsystems
    private Arm arm;
    private Shooter shooter;
    private LED led;
    private Supplier<Pose2d> poseSupplier;

    /**
     * Arms the robot to shoot into the SPEAKER.
     * Requires a NOTE to be LOADED.
     */
    public ArmSpeakerCommand(Arm arm, Shooter shooter, LED led, Supplier<Pose2d> poseSupplier) {
        this.arm = arm;
        this.shooter = shooter;
        this.led = led;
        this.poseSupplier = poseSupplier;

        addRequirements(arm, shooter, led);
    }

    // Exec
    @Override
    public void execute() {
        double dist = poseSupplier.get().getTranslation().getDistance(FieldConstants.kSPEAKER_POS);
        
        // Set shoot velocity
        shooter.setShootVelocity(ShootingRegressions.SHOOT_VELOCITY_FROM_DISTANCE(dist));

        // Set angle (trig for now)
        if (Math.abs(poseSupplier.get().getRotation().getDegrees()) <= 90.0) {
            // Robot is facing away from speaker
            arm.setExtension(1.0);
            arm.setLowerPivotAngle(15.0);
            arm.setUpperPivotAngle(180.0 - Units.radiansToDegrees(Math.atan(speakerToReversedPivotHeight / dist)));
        } else {
            // Robot is facing speaker
            arm.setExtension(1.0);
            arm.setUpperPivotAngle(0.0);
            arm.setLowerPivotAngle(Units.radiansToDegrees(Math.atan(speakerToPivotHeight / dist)));
        }

        // Set LEDs
        if (arm.atSetpoint() && shooter.atShootSetpoint()) { // TODO and heading
            led.setPulse(LEDConstants.kGREEN);
        } else {
            led.setAllianceStationColor();
        }
    }

    // Cancel self on interruption
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
