package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.SwerveBase;

public class TeleopDriveCommand extends Command {
    // Objects
    private SwerveBase base;

    // Inputs
    private Supplier<Double> ySupply, xSupply, thetaSupply;
    private Supplier<Boolean> fastMode, obstacleAvoidance;
    private boolean isThetaPercentage;

    // Filters
    //private SlewRateLimiter xLimiter = new SlewRateLimiter(2.0);
    //private SlewRateLimiter yLimiter = new SlewRateLimiter(2.0);
    //private SlewRateLimiter tLimiter = new SlewRateLimiter(2.0);

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
        Supplier<Boolean> fastMode,
        Supplier<Boolean> obstacleAvoidance) {
            this.base = base;
            this.xSupply = xSupply;
            this.ySupply = ySupply;
            this.thetaSupply = thetaSupply;
            this.isThetaPercentage = isThetaPercentage;
            this.fastMode = fastMode;
            this.obstacleAvoidance = obstacleAvoidance;
        
        addRequirements(base);
    }

    // Drive
    @Override
    public void execute() {
        // Fast mode?
        double max = (fastMode.get()) ? DriveConstants.kMAX_FAST_TELEOP_TRANSLATIONAL_SPEED : DriveConstants.kMAX_SLOW_TELEOP_TRANSLATIONAL_SPEED;

        /*base.driveFieldOriented(
            yLimiter.calculate(ySupply.get()) * max, 
            xLimiter.calculate(xSupply.get()) * max, 
            tLimiter.calculate(thetaSupply.get()) * (isThetaPercentage ? DriveConstants.kMAX_TELEOP_ROTATIONAL_SPEED : 1.0),
            obstacleAvoidance.get()
        );*/

        base.driveFieldOriented(
            MathUtil.applyDeadband(ySupply.get(), 0.05) * max, 
            MathUtil.applyDeadband(xSupply.get(), 0.05) * max, 
            MathUtil.applyDeadband(thetaSupply.get(), 0.05) * (isThetaPercentage ? DriveConstants.kMAX_TELEOP_ROTATIONAL_SPEED : 1.0),
            obstacleAvoidance.get()
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
