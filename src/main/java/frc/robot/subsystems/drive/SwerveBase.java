package frc.robot.subsystems.drive;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.frc7153.diagnostics.DiagUtil;
import com.frc7153.logging.LoggingUtil;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.struct.SwerveModuleStateStruct;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HardwareConstants;

public class SwerveBase extends SubsystemBase {
    // Shared SwerveModuleStateStruct
    private static SwerveModuleStateStruct kSwerveModuleState = new SwerveModuleStateStruct();

    // Modules
    private SwerveModule[] modules = {
        new SwerveModule( // FL
            HardwareConstants.kFL_DRIVE_CAN, HardwareConstants.kFL_STEER_CAN, 
            HardwareConstants.kFL_CANCODER, DriveConstants.kFL_STEER_ZERO
        ), new SwerveModule( // FR
            HardwareConstants.kFR_DRIVE_CAN, HardwareConstants.kFR_STEER_CAN, 
            HardwareConstants.kFR_CANCODER, DriveConstants.kFR_STEER_ZERO
        ), new SwerveModule( // RL
            HardwareConstants.kRL_DRIVE_CAN, HardwareConstants.kRL_STEER_CAN, 
            HardwareConstants.kRL_CANCODER, DriveConstants.kRL_STEER_ZERO
        ), new SwerveModule( // RR
            HardwareConstants.kRR_DRIVE_CAN, HardwareConstants.kRR_STEER_CAN, 
            HardwareConstants.kRR_CANCODER, DriveConstants.kRR_STEER_ZERO
        )
    };

    // Kinematics
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(DriveConstants.kSIZE.getX()/2.0, DriveConstants.kSIZE.getY()/2.0),  // FL
        new Translation2d(DriveConstants.kSIZE.getX()/2.0, DriveConstants.kSIZE.getY()/-2.0), // FR
        new Translation2d(DriveConstants.kSIZE.getX()/-2.0, DriveConstants.kSIZE.getY()/2.0), // RL
        new Translation2d(DriveConstants.kSIZE.getX()/-2.0, DriveConstants.kSIZE.getY()/-2.0) // RR
    );

    // Pose estimation
    private ADIS16470_IMU gyro = new ADIS16470_IMU(
        DriveConstants.kGYRO_YAW, DriveConstants.kGYRO_PITCH, 
        DriveConstants.kGYRO_ROLL, Port.kMXP, CalibrationTime._4s
    );
    private SwerveModulePosition[] modulePositions = {
        modules[0].getPosition(), modules[1].getPosition(), modules[2].getPosition(), modules[3].getPosition()
    };
    private SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(
        kinematics, 
        Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kYaw)), 
        modulePositions, 
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)), 
        DriveConstants.kSTATE_STD_DEV, 
        VecBuilder.fill(0.1, 0.1, 0.1) // this won't be used
    );
    private Vector<N3> visionStandardDev = VecBuilder.fill(0, 0, 0); // pre-allocated

    // Logs
    private StructArrayLogEntry<SwerveModuleState> setpointLog = 
        StructArrayLogEntry.create(DataLogManager.getLog(), "Drive/Swerve Setpoints", kSwerveModuleState, "m/s, rad");
    private StructArrayLogEntry<SwerveModuleState> stateLog = 
        StructArrayLogEntry.create(DataLogManager.getLog(), "Drive/Swerve States", kSwerveModuleState, "m/s, rad");
    private StructLogEntry<Pose2d> poseLog =
        StructLogEntry.create(DataLogManager.getLog(), "Drive/Pose Estimation", new Pose2dStruct());
    private DoubleLogEntry avgAmbiguityLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Drive/Tag Ambiguity (avg)", "Should not exceed 0.2");
    private IntegerLogEntry tagIdLog = 
        new IntegerLogEntry(DataLogManager.getLog(), "Drive/Tag IDs");

    // Module state arrays (for logging)
    private SwerveModuleState[] setpointArray = new SwerveModuleState[4];
    private SwerveModuleState[] stateArray = new SwerveModuleState[4];

    // Constructor
    public SwerveBase() {
        // Start logging
        DiagUtil.addDevice(gyro);
    }

    /**
     * Drives the robot from an array of 4 swerve module states
     * @param speeds Array of states (FL, FR, RL, RR)
     */
    public void drive(SwerveModuleState[] states) {
        if (states.length != 4) {
            LoggingUtil.warn("Could not set swerve module states because did not receive 4 (received %d)", states.length);
            return;
        }

        for (int s = 0; s < 4; s++) {
            modules[s].setModuleState(states[s]);
        }
    }

    /**
     * Drives the robot, field oriented
     * @param x Forward/backward (m/s)
     * @param y Left/Right (m/s)
     * @param theta Rotation (degrees/second, CCW+)
     */
    public void driveFieldOriented(double x, double y, double theta) {
        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
            x, 
            y, 
            Units.degreesToRadians(theta), 
            estimator.getEstimatedPosition().getRotation()
        );

        drive(kinematics.toSwerveModuleStates(chassisSpeed));
    }

    /**
     * Resets the pose estimator's position. 
     * (Call this at the beginning of autonomous)
     * @param pos
     */
    public void resetPosition(Pose2d pos) {
        // Repopulate module positions array
        for (int m = 0; m < 4; m++) {
            modulePositions[m] = modules[m].getPosition();
        }

        estimator.resetPosition(Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kYaw)), modulePositions, pos);
    }

    /**
     * Adds a vision measurement to the pose estimator
     * @param pose The EstimatedRobotPose from the camera
     * @return Success
     */
    public boolean addVisionMeasurement(EstimatedRobotPose pose) {
        if (pose.targetsUsed.size() == 0) {
            // No targets here!
            return false;
        } else if (pose.targetsUsed.size() == 1) {
            // Only 1 target -> default distribution
            PhotonTrackedTarget tag = pose.targetsUsed.get(0);

            if (tag.getPoseAmbiguity() == -1 || tag.getPoseAmbiguity() > 0.2 || tag.getFiducialId() == -1) {
                // This tag is bogus
                LoggingUtil.warn("Invalid tag found in EstimatedRobotPose!");
                return false;
            }

            visionStandardDev.set(0, 0, DriveConstants.kDEFAULT_VISION_XY_STD_DEV);
            visionStandardDev.set(1, 0, DriveConstants.kDEFAULT_VISION_XY_STD_DEV);
            visionStandardDev.set(2, 0, DriveConstants.kDEFAULT_VISION_RAD_STD_DEV);

            // A histogram can be created to see which tags are seen the most
            tagIdLog.append(tag.getFiducialId());
            avgAmbiguityLog.append(tag.getPoseAmbiguity());
        } else {
            // Many tags, find the standard deviation
            double minX = pose.targetsUsed.get(0).getBestCameraToTarget().getX();
            double maxX = pose.targetsUsed.get(0).getBestCameraToTarget().getX();
            double minY = pose.targetsUsed.get(0).getBestCameraToTarget().getY();
            double maxY = pose.targetsUsed.get(0).getBestCameraToTarget().getY();
            double minR = pose.targetsUsed.get(0).getBestCameraToTarget().getRotation().getZ();
            double maxR = pose.targetsUsed.get(0).getBestCameraToTarget().getRotation().getZ();
            double avgAmbiguity = 0.0; // IDK what to do with ambiguity.

            for (PhotonTrackedTarget tag : pose.targetsUsed) {
                if (tag.getPoseAmbiguity() == -1 || tag.getFiducialId() == -1) {
                    // Bogus tags
                    LoggingUtil.warn("Invalid tag found in EstimatedRobotPose!");
                    return false;
                }

                // Recalculate std dev
                minX = Math.min(minX, tag.getBestCameraToTarget().getX());
                maxX = Math.min(minX, tag.getBestCameraToTarget().getX());
                minY = Math.min(minX, tag.getBestCameraToTarget().getY());
                maxY = Math.min(minX, tag.getBestCameraToTarget().getY());
                minR = Math.min(minX, tag.getBestCameraToTarget().getRotation().getZ());
                maxR = Math.min(minX, tag.getBestCameraToTarget().getRotation().getZ());
                avgAmbiguity += tag.getPoseAmbiguity();

                // Log
                tagIdLog.append(tag.getFiducialId());
                avgAmbiguityLog.append(avgAmbiguity);
            }

            // Calculate the distributions
            avgAmbiguity /= pose.targetsUsed.size();
            visionStandardDev.set(0, 0, Math.max(maxX - minX, 0.076));
            visionStandardDev.set(1, 0, Math.max(maxY - minY, 0.076));
            visionStandardDev.set(2, 0, Math.max(maxR - minR, 0.14));
        }

        Pose2d visionPos = pose.estimatedPose.toPose2d();

        // Check how far off this is
        if (visionPos.getTranslation().getDistance(estimator.getEstimatedPosition().getTranslation()) > 1.3) {
            // More than 4ft off...
            LoggingUtil.warn("Received vision pose that was far (>1.3m) off");
            return false;
        }

        // Add this measurement
        estimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, visionStandardDev);
        return true;
    }

    /**
     * Returns the estimator's current position
     * @return
     */
    public Pose2d getPosition() { return estimator.getEstimatedPosition(); }

    // Periodic method
    @Override
    public void periodic() {
        for (int m = 0; m < 4; m++) {
            // Run each module's periodic
            modules[m].periodic();

            // Get states
            setpointArray[m] = modules[m].getSetpoint();
            stateArray[m] = modules[m].getState();
            modulePositions[m] = modules[m].getPosition();
        }

        // Update pose estimator
        estimator.update(Rotation2d.fromDegrees(gyro.getAngle(IMUAxis.kYaw)), modulePositions);

        // Log
        setpointLog.append(setpointArray);
        stateLog.append(stateArray);
        poseLog.append(estimator.getEstimatedPosition());
    }
}
