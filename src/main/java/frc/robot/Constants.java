package frc.robot;

import java.nio.file.Path;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.subsystems.Arm.ArmState;

/**
 * Robot constants. See SwerveConstants.java for constants relating to the
 * Swerve Base
 */
public class Constants {
    /** Build Constants */
    public static final class BuildConstants {
        public static final boolean kOUTPUT_ALL_TELEMETRY = true; // For debugging, output all live values to NT
        public static final boolean kARM_TUNE_MODE = false; // For tuning lower pivot PID

        public static final boolean kDRIVE_TUNE_MODE = false; // For tuning drive base PID

        public static final boolean kARM_MOTION_PROFILING = false; // Using trapezoidal motion profile for arm?

        public static final boolean kSECONDARY_INTAKE_MOTOR = false; // If secondary motor is attacked
    }

    /** Shooter Constants */
    public static final class ShooterConstants {
        public static final int kSHOOT_CURRENT_LIMIT = 55;
        public static final double kSHOOT_RATIO = 30.0 / 18.0; // step-up ratio
        public static final double kSHOOT_P = 0.5;
        public static final double kSHOOT_I = 0.0;
        public static final double kSHOOT_D = 0.0;
        public static final double kSHOOT_TOLERANCE = 5.0;

        public static final int kINDEXER_CURRENT_LIMIT = 30;
        public static final double kINDEXER_RATIO = 1.0 / 5.0;
        public static final double kINDEXER_VELO_P = 0.000005;
        public static final double kINDEXER_VELO_I = 5e-7;
        public static final double kINDEXER_VELO_D = 0.0;
    }

    /**Intake Constants */
    public static final class IntakeConstants {
        public static final double kINTAKE_RATIO = 1.0 / 25.0;
        
        public static final int kINTAKE_CURRENT_LIMIT = 40;

        public static final double kINTAKE_P = 0.000005;
        public static final double kINTAKE_I = 5e-7;
        public static final double kINTAKE_D = 0.0;
    }
    /** Secondary Intake Constants */
    public static final class SecondaryIntakeConstants {
        public static final double kINTAKE_RATIO = 1.0 / 25.0;

        public static final int kINTAKE_CURRENT_LIMIT = 30;

        public static final double kINTAKE_P = 0.000005;
        public static final double kINTAKE_I = 5e-7;
        public static final double kINTAKE_D = 0.0;
    }
    /** Climber Constants */
    public static final class ClimberConstants {
        public static final double kCLIMBER_RATIO = 1.0 / 25.0;
        public static final int kCLIMBER_CURRENT_LIMIT = 40; 

        public static final double kCLIMBER_P = 0.2;
        public static final double kCLIMBER_I = 0.0;
        public static final double kCLIMBER_D = 0.0;

        public static final double kCLIMB_VELO_P = 0.00005;
        public static final double kCLIMB_VELO_I = 0.0;
        public static final double kCLIMB_VELO_D = 0.0;

        public static final double kCLIMB_ROLL_STABILIZE_P = 0.01; // todo

        public static final double kCLIMBER_TOLERANCE = 3.0;
    }

    /** Arm Preset Positions */
    public static final class ArmPositions {
        public static final ArmState kDEFAULT = new ArmState(108.0, 180.0, 0.0);

        public static final ArmState kGROUND_INTAKE = new ArmState(110.0, 255.0, 0.75); // 0.8
        public static final ArmState kSOURCE_INTAKE_FRONT = new ArmState(135.0, 180.0, 1.0);

        public static final ArmState kSUBWOOFER_SPEAKER_FRONT = new ArmState(120.0, 200.0, 0.0);
        public static final ArmState kSUBWOOFER_SPEAKER_REAR = new ArmState(125.0, 270.0, 0.0);

        public static final ArmState kFRONT_AMP = new ArmState(145.0, 275.0, 2.5);
        public static final ArmState kREAR_AMP = new ArmState(190.0, 105.0, 1.5);

        public static final ArmState kCLIMB_BALANCE = new ArmState(108.0, 180.0, 1.5);

        public static final ArmState kFULL_COURT_PASS = kSUBWOOFER_SPEAKER_FRONT;
    }

    /** Auto Constants */
    public static final class AutoConstants {
        public static final HolonomicPathFollowerConfig kAUTO_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 4.65, 0.0), 
            new PIDConstants(6.5, 2.0, 0.0), 
            4.0, 
            DriveConstants.kRL_SWERVE_POS.getNorm(), 
            new ReplanningConfig()
        );
    }

    /** Shooting Calculations */
    public static final class ShootingRegressions {
        /** Determine shooter angle from rear limelight tag distance (m) */
        public static final ArmState V1_SHOOT_ANGLE_FROM_REAR_LIMELIGHT_DISTANCE(double distance) {
            // See: https://www.desmos.com/calculator/0cghhlhgfi 

            distance = MathUtil.clamp(distance, 0.0, 4.0);
            return new ArmState(
                120.0, 
                (-17.017 * distance * distance) + (85.3966 * distance) + 183.461,
                0.0
            );
        }

        /** Determine shooter angle from rear limelight tag distance (m) */
        public static final ArmState V2_SHOOT_ANGLE_FROM_REAR_LIMELIGHT_DISTANCE(double distance) {
            ArmState state = V1_SHOOT_ANGLE_FROM_REAR_LIMELIGHT_DISTANCE(distance);
            state.upperAngle = state.upperAngle + 5.0;
            return state;
        }

        /** Determine upper shooter angle from front limelight's tag distance (m) */
        public static final double LIMELIGHT_REGRESSION_V3(double d) {
            return 4.55516 * d * d + -41.7191 * d + 254.806 + 0.5;
        }
    }

    /** Arm Constants */
    public static final class ArmConstants {
        public static final int kLOWER_RIGHT_PIVOT_CURRENT_LIMIT = 60; // 60
        public static final int kLOWER_LEFT_PIVOT_CURRENT_LIMIT = 60; // 60
        public static final int kUPPER_PIVOT_CURRENT_LIMIT = 15; // 25
        public static final int kELEVATOR_EXT_CURRENT_LIMIT = 15; // 35

        public static final Constraints kLOWER_PIVOT_TRAPEZOID_CONTROL = new Constraints(45.0, 15.0);

        public static final double kLOWER_ANGLE_OFFSET = 0.18 + 0.5; // rots
        public static final double kUPPER_ANGLE_OFFSET = 0.3356208; // rots 0.5 = forward

        public static final double kLOWER_PIVOT_RATIO = 1.0 / (4.0 * 3.0 * 3.0 * (60.0/18.0));
        public static final double kUPPER_PIVOT_RATIO = 1.0 / 16.0;
        public static final double kELEVATOR_EXT_RATIO = 1.0 / 25.0;

        public static final double kLOWER_PIVOT_P = 3.3; // 3.15
        public static final double kLOWER_PIVOT_I = 0.001;
        public static final double kLOWER_PIVOT_D = 0.0;
        public static final double kLOWER_PIVOT_FF = 0.00001;

        public static final double kUPPER_PIVOT_P = 1.55; // 1.7 oscillates 
        public static final double kUPPER_PIVOT_I = 0.0004; 
        public static final double kUPPER_PIVOT_D = 0.0;//001; 

        public static final double kELEVATOR_EXT_P = 0.03;
        public static final double kELEVATOR_EXT_I = 0.0;
        public static final double kELEVATOR_EXT_D = 0.0;

        public static final double kLOWER_ANGLE_TOLERANCE = 7.0; //deg TODO these are guessed
        public static final double kUPPER_ANGLE_TOLERANCE = 5.0; //deg
        public static final double kEXT_TOLERANCE = 0.25; //rots
    }

    /** Swerve drive constants */
    public static final class DriveConstants {
        // Max Speeds
        public static final double kMAX_SLOW_TELEOP_TRANSLATIONAL_SPEED = 5.0; // 5.0 for comp, 11.5 for non-comp
        public static final double kMAX_FAST_TELEOP_TRANSLATIONAL_SPEED = 15.5; // 15.5 for comp, 13.5 for non-comp
        public static final double kMAX_TELEOP_ROTATIONAL_SPEED = 800.0;

        // Base sizes
        /*
         * The X and Y values are implemented in WPI's library oddly:
         * "Positive x values represent moving toward the front of the robot whereas positive 
         * y values represent moving toward the left of the robot."
         * 
         * See https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html 
         */
        public static final Translation2d kFL_SWERVE_POS = new Translation2d(Units.inchesToMeters(5.875), Units.inchesToMeters(12.375));
        public static final Translation2d kFR_SWERVE_POS = new Translation2d(Units.inchesToMeters(5.875), Units.inchesToMeters(-12.375));
        public static final Translation2d kRL_SWERVE_POS = new Translation2d(Units.inchesToMeters(-11.375), Units.inchesToMeters(12.375));
        public static final Translation2d kRR_SWERVE_POS = new Translation2d(Units.inchesToMeters(-11.375), Units.inchesToMeters(-12.375));

        public static final double kARM_LOWER_PIVOT_HEIGHT = Units.inchesToMeters(13.233 + 0.62);//TODO: Move to arm constants // Pivot to ground

        // CANCoder Offsets
        public static final double kFL_STEER_ZERO = 0.125448; // 14
        public static final double kFR_STEER_ZERO = -0.321777; // 17
        public static final double kRL_STEER_ZERO = 0.155762; // 15
        public static final double kRR_STEER_ZERO = -0.181152; // 16

        // Pose Estimation
        public static final IMUAxis kGYRO_YAW = IMUAxis.kZ; 
        public static final IMUAxis kGYRO_PITCH = IMUAxis.kX;
        public static final IMUAxis kGYRO_ROLL = IMUAxis.kY; 
        
        public static final Vector<N3> kSTATE_STD_DEV = 
            VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(2.0)); // meters, meters, rads
        public static final Vector<N3> kVISION_STD_DEV =
            VecBuilder.fill(0.075, 0.075, Units.degreesToRadians(4.0));

        public static final Path kAPRIL_TAG_LAYOUT_JSON = Filesystem.getDeployDirectory().toPath().resolve("AprilTag2024Layout.json");

        // Speaker Heading Correction PID
        public static final double kHEADING_CORRECTION_P = 1.85;
        public static final double kHEADING_CORRECTION_I = 0.2;
    }

    /** Swerve module constants */
    public static final class SwerveModuleConstants {
        public static final double kWHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI; // meters

        // Drive Motor
        public static final double kDRIVE_RATIO = 6.75; // SDS Mk4i L2 modules
        public static final int kDRIVE_CURRENT_LIMIT = 45;
        public static final double kDRIVE_P = 0.00025; // 0.00025
        public static final double kDRIVE_I = 0.0; // 0.0000005
        public static final double kDRIVE_D = 0.0;
        public static final double kDRIVE_FF = 0.000025;

        // Steer Motor
        public static final double kSTEER_RATIO = 150.0 / 7.0;
        public static final int kSTEER_CURRENT_LIMIT = 25;
        public static final double kSTEER_P = 0.45;
        public static final double kSTEER_I = 0.00001;
        public static final double kSTEER_D = 0.0;
        public static final double kSTEER_FF = 0.0;
    }

    /** CAN Hardware Constants */
    public static final class HardwareConstants {
        public static final int kPDH_CAN = 1;

        public static final int kFL_STEER_CAN = 2;
        public static final int kFR_STEER_CAN = 3;
        public static final int kRL_STEER_CAN = 4;
        public static final int kRR_STEER_CAN = 5;

        public static final int kFL_DRIVE_CAN = 6;
        public static final int kFR_DRIVE_CAN = 7;
        public static final int kRL_DRIVE_CAN = 8;
        public static final int kRR_DRIVE_CAN = 9;

        public static final int kINTAKE_CAN = 10;
        public static final int kINDEXER_CAN = 11;

        public static final int kLOWER_LEFT_PIVOT_CAN = 13;
        public static final int kLOWER_RIGHT_PIVOT_CAN = 12;
 
        public static final int kFL_CANCODER = 14;
        public static final int kFR_CANCODER = 17;
        public static final int kRL_CANCODER = 15;
        public static final int kRR_CANCODER = 16;

        public static final int kSHOOTER_UPPER_CAN = 19;
        public static final int kSHOOTER_LOWER_CAN = 18;

        public static final int kCLIMBER_LEFT_CAN = 21;
        public static final int kCLIMBER_RIGHT_CAN = 20; 

        public static final int kUPPER_PIVOT_CAN = 22;

        public static final int kELEVATOR_EXT_CAN = 23;

        public static final String kCANIVORE_BUS = "CANivore"; // CANivore

        public static final int kINTAKE_SECONDARY = 24;
    }

    /** Field Constants */
    public static final class FieldConstants {
        public static final Translation2d kFIELD_SIZE = new Translation2d(
            Units.inchesToMeters((54.0 * 12.0) + 3.25), 
            Units.inchesToMeters((26.0 * 12.0) + 11.25)
        );

        /**
         * Pose of the speaker, meters
         */
        public static final Translation2d kSPEAKER_POS = new Translation2d(0.0, 5.547867999999999);

        // Average of lowest and highest point (meters):
        public static final double kSPEAKER_HEIGHT = Units.inchesToMeters((((6.0 * 12.0) + 6.0) + ((6.0 * 12.0) * 10.875))/2.0);

        /** Inverts the alliance of alliance-relative Pose2d */
        public static final Pose2d INVERT_ALLIANCE(Pose2d original) {
            return new Pose2d(
                (kFIELD_SIZE.getX()) - original.getX(), // X inverted around midline
                original.getY(), // Same Y
                new Rotation2d( // Invert angle around midline
                    -original.getRotation().getCos(), 
                    original.getRotation().getSin()
                )
                //original.getRotation()
            );
        }
    }
}


