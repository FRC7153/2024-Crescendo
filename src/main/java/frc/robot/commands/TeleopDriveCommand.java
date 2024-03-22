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
    private Supplier<Boolean> fastMode;
    private boolean isThetaPercentage;

    /**
     * Teleop driving command for the swerve base
     * @param base
     * @param ySupply %, forward/backward
     * @param xSupply %, left/right
     * @param thetaSupply CCW+, see below
     * @param isThetaPercentage if true, theta is % of max rotational speed, else theta is deg/sec
     * @param fastMode if true, increases drive speed
     */
    public TeleopDriveCommand(
        SwerveBase base, 
        Supplier<Double> ySupply, 
        Supplier<Double> xSupply, 
        Supplier<Double> thetaSupply,
        boolean isThetaPercentage,
        Supplier<Boolean> fastMode) {
            this.base = base;
            this.xSupply = xSupply;
            this.ySupply = ySupply;
            this.thetaSupply = thetaSupply;
            this.isThetaPercentage = isThetaPercentage;
            this.fastMode = fastMode;
        
        addRequirements(base);
    }

    // Drive
    @Override
    public void execute() {
        // Fast mode?
        double max = (fastMode.get()) ? DriveConstants.kMAX_FAST_TELEOP_TRANSLATIONAL_SPEED : DriveConstants.kMAX_SLOW_TELEOP_TRANSLATIONAL_SPEED;

        base.driveFieldOriented(
            ySupply.get() * max, 
            xSupply.get() * max, 
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
