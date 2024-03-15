package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
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
    }

    /** Shooter Constants */
    public static final class ShooterConstants {
        public static final int kSHOOT_CURRENT_LIMIT = 60;
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
        
        public static final int kINTAKE_CURRENT_LIMIT = 15;

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

        public static final double kCLIMBER_TOLERANCE = 3.0;
    }

    /** Arm Preset Positions */
    public static final class ArmPositions {
        public static final ArmState kDEFAULT = new ArmState(108.0, 180.0, 0.0);

        public static final ArmState kGROUND_INTAKE = new ArmState(110.0, 255.0, 0.5); // 110

        public static final ArmState kSUBWOOFER_SPEAKER_FRONT = new ArmState(120.0, 200.0, 0.0);
        public static final ArmState kSUBWOOFER_SPEAKER_REAR = new ArmState(125.0, 270.0, 0.0);

        public static final ArmState kFRONT_AMP = new ArmState(145.0, 275.0, 2.5);
        public static final ArmState kREAR_AMP = new ArmState(190.0, 105.0, 1.5);

        public static final ArmState kCLIMB_BALANCE = new ArmState(108.0, 180.0, 1.5);

    }

    /** Shooting Calculations */
    public static final class ShootingRegressions {
        /** Determine shoot velocity from distance (m) */
        public static final double SHOOT_VELOCITY_FROM_DISTANCE(double distance) {
            return 100.0; // TODO regression
        }
    }

    /** Arm Constants */
    public static final class ArmConstants {
        public static final int kLOWER_RIGHT_PIVOT_CURRENT_LIMIT = 60; // 60
        public static final int kLOWER_LEFT_PIVOT_CURRENT_LIMIT = 60; // 60
        public static final int kUPPER_PIVOT_CURRENT_LIMIT = 15; // 25
        public static final int kELEVATOR_EXT_CURRENT_LIMIT = 15; // 35

        public static final double kLOWER_ANGLE_OFFSET = 0.163; // rots
        public static final double kUPPER_ANGLE_OFFSET = 0.3356208; // rots 0.5 = forward

        /** rots, minimum angle of lower pivot for upper pivot to safely spin */
        public static final double kUPPER_PIVOT_MIN_ARM_ANGLE = 108.0 / 360.0; // rots

        public static final double kLOWER_PIVOT_RATIO = 1.0 / (4.0 * 3.0 * 3.0 * (60.0/18.0));
        public static final double kUPPER_PIVOT_RATIO = 1.0 / 16.0;
        public static final double kELEVATOR_EXT_RATIO = 1.0 / 25.0;

        public static final double kLOWER_PIVOT_P = 3.15;
        public static final double kLOWER_PIVOT_I = 0.001;
        public static final double kLOWER_PIVOT_D = 0.0;
        public static final double kLOWER_PIVOT_FF = 0.00001;

        public static final double kUPPER_PIVOT_P = 1.0; // 0.1 while motor's relative
        public static final double kUPPER_PIVOT_I = 0.00001; 
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
        public static final double kMAX_TELEOP_TRANSLATIONAL_SPEED = 15;
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
        public static final double kFL_STEER_ZERO = 0.35969 - 0.5; // 0.497314
        public static final double kFR_STEER_ZERO = 0.11376; // -0.386719
        public static final double kRL_STEER_ZERO = -0.1297; // 0.2769 - 0.5
        public static final double kRR_STEER_ZERO = -0.332275 + 0.5; // -0.332275

        // Pose Estimation
        public static final IMUAxis kGYRO_YAW = IMUAxis.kZ; 
        public static final IMUAxis kGYRO_PITCH = IMUAxis.kX;
        public static final IMUAxis kGYRO_ROLL = IMUAxis.kY; 
        
        public static final Vector<N3> kSTATE_STD_DEV = 
            VecBuilder.fill(0.8, 0.8, Units.degreesToRadians(2.0)); // meters, meters, rads
        public static final Vector<N3> kVISION_STD_DEV =
            VecBuilder.fill(0.075, 0.075, Units.degreesToRadians(4.0));

        public static final Path kAPRIL_TAG_LAYOUT_JSON = Filesystem.getDeployDirectory().toPath().resolve("AprilTag2024Layout.json");

        // Heading Correction PID
        public static final double kHEADING_CORRECTION_P = 0.5; // TODO
        public static final double kHEADING_CORRECTION_I = 0.0; // TODO
        public static final double kHEADING_CORRECTION_D = 0.0; // TODO        
    }

    /** Swerve module constants */
    public static final class SwerveModuleConstants {
        public static final double kWHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI; // meters

        // Drive Motor
        public static final double kDRIVE_RATIO = 6.75; // SDS Mk4i L2 modules
        public static final int kDRIVE_CURRENT_LIMIT = 50;
        public static final double kDRIVE_P = 0.00005;
        public static final double kDRIVE_I = 0.0;
        public static final double kDRIVE_D = 0.0; // 5e-7
        public static final double kDRIVE_S = 0.0; // TODO
        public static final double kDRIVE_V = 0.0; // TODO
        public static final double kDRIVE_A = 0.0; // TODO

        // Steer Motor
        public static final double kSTEER_RATIO = 150.0 / 7.0;
        public static final int kSTEER_CURRENT_LIMIT = 30;
        public static final double kSTEER_P = 0.45;
        public static final double kSTEER_I = 0.00001;
        public static final double kSTEER_D = 0.0;
        public static final double kSTEER_FF = 0.0; // TODO
    }

    /** CAN Hardware Constants */
    public static final class HardwareConstants { // TODO verify all these
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
    }

    /** LED Constants */
    public static final class LEDConstants {
        public static final double kOFF = 0.99;
        public static final double kRED = 0.61;
        public static final double kBLUE = 0.87;
        public static final double kGREEN = 0.77;
        public static final double kYELLOW = 0.69;
    }

    /** Field Constants */
    public static final class FieldConstants {
        public static final Translation2d kFIELD_SIZE = new Translation2d(
            Units.inchesToMeters((54.0 * 12.0) + 3.25), 
            Units.inchesToMeters((26.0 * 12.0) + 11.25)
        );

        public static final Translation2d kSPEAKER_POS = new Translation2d(0.0, 5.547867999999999);

        // Average of lowest and highest point (meters):
        public static final double kSPEAKER_HEIGHT = Units.inchesToMeters((((6.0 * 12.0) + 6.0) + ((6.0 * 12.0) * 10.875))/2.0);

        /** Inverts the alliance of alliance-relative Pose2d */
        public static final Pose2d INVERT_ALLIANCE(Pose2d original) {
            return new Pose2d(
                (kFIELD_SIZE.getX()) - original.getX(), // X inverted around midline
                original.getY(), // Same Y
                new Rotation2d( // Invert angle around midline
                    original.getRotation().getCos(), 
                    -original.getRotation().getSin()
                )
            );
        }
    }
}


