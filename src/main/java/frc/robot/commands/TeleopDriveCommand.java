package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.SwerveBase;

public class TeleopDriveCommand extends Command {
    // Objects
    private SwerveBase base;

    // Inputs
    private Supplier<Double> ySupply, xSupply, thetaSupply;

    /**
     * Teleop driving command for the swerve base
     */
    public TeleopDriveCommand(
        SwerveBase base, 
        Supplier<Double> ySupply, 
        Supplier<Double> xSupply, 
        Supplier<Double> thetaSupply) {
            this.base = base;
            this.xSupply = xSupply;
            this.ySupply = ySupply;
            this.thetaSupply = thetaSupply;
        
        addRequirements(base);
    }

    // Drive
    @Override
    public void execute() {
        base.driveFieldOriented(
            ySupply.get() * DriveConstants.kMAX_TELEOP_TRANSLATIONAL_SPEED, 
            xSupply.get() * DriveConstants.kMAX_TELEOP_TRANSLATIONAL_SPEED, 
            thetaSupply.get() * DriveConstants.kMAX_TELEOP_ROTATIONAL_SPEED
        );
    }

    // End
    @Override
    public void end(boolean terminated) {
        base.driveFieldOriented(0.0, 0.0, 0.0);
    }

    // Cancel self
    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}
