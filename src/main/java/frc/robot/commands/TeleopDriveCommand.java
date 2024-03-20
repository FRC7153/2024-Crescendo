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
    private boolean isThetaPercentage;

    /**
     * Teleop driving command for the swerve base
     * @param base
     * @param ySupply %, forward/backward
     * @param xSupply %, left/right
     * @param thetaSupply CCW+, see below
     * @param isThetaPercentage if true, theta is % of max rotational speed, else theta is deg/sec
     */
    public TeleopDriveCommand(
        SwerveBase base, 
        Supplier<Double> ySupply, 
        Supplier<Double> xSupply, 
        Supplier<Double> thetaSupply,
        boolean isThetaPercentage) {
            this.base = base;
            this.xSupply = xSupply;
            this.ySupply = ySupply;
            this.thetaSupply = thetaSupply;
            this.isThetaPercentage = isThetaPercentage;
        
        addRequirements(base);
    }

    // Drive
    @Override
    public void execute() {
        base.driveFieldOriented(
            ySupply.get() * DriveConstants.kMAX_TELEOP_TRANSLATIONAL_SPEED, 
            xSupply.get() * DriveConstants.kMAX_TELEOP_TRANSLATIONAL_SPEED, 
            thetaSupply.get() * (isThetaPercentage ? DriveConstants.kMAX_TELEOP_ROTATIONAL_SPEED : 1.0)
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
