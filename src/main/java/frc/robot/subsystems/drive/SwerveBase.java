package frc.robot.subsystems.drive;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.frc7153.diagnostics.DiagUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.struct.Pose2dStruct;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.struct.SwerveModuleStateStruct;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.util.Util;

public class SwerveBase implements Subsystem {
    // Shared SwerveModuleStateStruct
    private static SwerveModuleStateStruct kSwerveModuleState = new SwerveModuleStateStruct();

    // Preallocated Alliance Optional
    private Optional<Alliance> alliance;

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
        DriveConstants.kFL_SWERVE_POS,  // FL
        DriveConstants.kFR_SWERVE_POS, // FR
        DriveConstants.kRL_SWERVE_POS, // RL
        DriveConstants.kRR_SWERVE_POS // RR
    );

    // Pose estimation
    private ADIS16470_IMU gyro = new ADIS16470_IMU(
        DriveConstants.kGYRO_YAW, DriveConstants.kGYRO_PITCH, 
        DriveConstants.kGYRO_ROLL, SPI.Port.kOnboardCS0, CalibrationTime._4s
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
        DriveConstants.kVISION_STD_DEV
    );

    // Logs
    private StructArrayLogEntry<SwerveModuleState> setpointLog = 
        StructArrayLogEntry.create(DataLogManager.getLog(), "Drive/Swerve Setpoints", kSwerveModuleState, "m/s, rad");
    private StructArrayLogEntry<SwerveModuleState> stateLog = 
        StructArrayLogEntry.create(DataLogManager.getLog(), "Drive/Swerve States", kSwerveModuleState, "m/s, rad");
    private StructLogEntry<Pose2d> poseLog =
        StructLogEntry.create(DataLogManager.getLog(), "Drive/Pose Estimation", new Pose2dStruct());
    private DoubleLogEntry ambiguityLog = 
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
        register();
    }

    /** Sets the default command (doesn't move) */
    public void initDefaultCommand() {
        setDefaultCommand(new InstantCommand(
            () -> driveFieldOriented(0.0, 0.0, 0.0),
            this
        ));
    }

    /**
     * Drives the robot from an array of 4 swerve module states
     * @param speeds Array of states (FL, FR, RL, RR)
     */
    public void drive(SwerveModuleState[] states) {
        if (states.length != 4) {
            DriverStation.reportWarning(String.format(
                "Could not set swerve module states because did not receive 4 (received %d)", 
                states.length
            ), false);
            return;
        }

        for (int s = 0; s < 4; s++) {
            modules[s].setModuleState(states[s]);
        }
    }

    /**
     * Drives the robot, field oriented
     * @param y Forward + / backward - (m/s)
     * @param x Left - /Right + (m/s)
     * @param theta Rotation (degrees/second, CCW+)
     */
    public void driveFieldOriented(double y, double x, double theta) {
        ChassisSpeeds chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(
            y, 
            -x, 
            Units.degreesToRadians(theta), 
            getYaw()
        );

        drive(kinematics.toSwerveModuleStates(chassisSpeed));
    }

    /**
     * Drives the robot, field oriented. Positive theta will always rotate the robot towards the AMPs
     * @param y
     * @param x
     * @param theta
     */
    public void driveFullFieldOriented(double y, double x, double theta) {
        alliance = DriverStation.getAlliance();

        if (alliance.isPresent() && alliance.get().equals(Alliance.Red)) {
            Rotation2d rot = new Rotation2d(theta);
            theta = (new Rotation2d(-rot.getCos(), rot.getSin())).getDegrees();
        }

        driveFieldOriented(y, x, theta);
    }

    /**
     * Resets the pose estimator and gyro's position. Will account for alliance. 
     * (Call this at the beginning of autonomous)
     * @param pos
     */
    public void resetPosition(Pose2d pos) {
        // Repopulate module positions array
        for (int m = 0; m < 4; m++) {
            modulePositions[m] = modules[m].getPosition();
        }

        // Account for red alliance
        alliance = DriverStation.getAlliance();

        if (!alliance.isPresent()) {
            DriverStation.reportWarning("Attempted to reset position with no known alliance!", false);
        } else if (alliance.get().equals(Alliance.Red)) {
            pos = FieldConstants.INVERT_ALLIANCE(pos);
        }

        gyro.setGyroAngle(DriveConstants.kGYRO_YAW, pos.getRotation().getDegrees());
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
        }

        Pose2d visionPos = pose.estimatedPose.toPose2d();

        // Check how far off this is
        if (visionPos.getTranslation().getDistance(estimator.getEstimatedPosition().getTranslation()) > 1.3) {
            // More than 4ft off...
            DriverStation.reportWarning("Received vision pose that was far (>1.3m) off", false);
            return false;
        }

        // Add this measurement
        estimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);

        // Log
        for (PhotonTrackedTarget tag : pose.targetsUsed) {
            ambiguityLog.append(tag.getPoseAmbiguity());
            tagIdLog.append(tag.getFiducialId());
        }

        return true;
    }

    /**
     * Returns the estimator's current position
     * @param global if true, position will always be relative to blue alliance. If false, it will
     * be relative to the robot's alliance
     * @return
     */
    public Pose2d getPosition(boolean global) {
        Pose2d estimation = estimator.getEstimatedPosition();

        // Invert if red
        if (!global && Util.isRedAlliance()) {
            return FieldConstants.INVERT_ALLIANCE(estimation);
        }

        return estimation;
    }

    /**
     * @return Yaw, CCW+
     */
    public Rotation2d getYaw() {
        //System.out.printf("YAW -> %f\n", MathUtil.inputModulus(gyro.getAngle(IMUAxis.kYaw), 0.0, 360.0));
        return Rotation2d.fromDegrees(MathUtil.inputModulus(gyro.getAngle(IMUAxis.kYaw), 0.0, 360.0));
    }

    // TEMP
    private StructArrayPublisher<SwerveModuleState> statePub = NetworkTableInstance.getDefault().getTable("DriveTemp").getStructArrayTopic("States", kSwerveModuleState).publish();
    private StructArrayPublisher<SwerveModuleState> setpointPub = NetworkTableInstance.getDefault().getTable("DriveTemp").getStructArrayTopic("Setpoints", kSwerveModuleState).publish();

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

        //System.out.printf("X: %f, Y: %f, Z: %f\n", gyro.getAngle(IMUAxis.kX), gyro.getAngle(IMUAxis.kY), gyro.getAngle(IMUAxis.kZ));

        // Log
        setpointLog.append(setpointArray);
        stateLog.append(stateArray);
        poseLog.append(estimator.getEstimatedPosition());

        // Temp
        statePub.set(stateArray);
        setpointPub.set(setpointArray);
    }
}
