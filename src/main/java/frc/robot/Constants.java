package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

/**
 * Robot constants. See SwerveConstants.java for constants relating to the
 * Swerve Base
 */
public class Constants {
    /** Shooter Constants */
    public static class ShooterConstants {
        public static int kSHOOT_CURRENT_LIMIT = 60;
        public static double kSHOOT_RATIO = 30.0 / 18.0; // step-up ratio
        public static double kSHOOT_P = 0.5; //2.347e-7;
        public static double kSHOOT_I = 0.0;
        public static double kSHOOT_D = 0.0;
        public static double kSHOOT_TOLERANCE = 5.0; // Sys ID gives max velo error 26.7 deg/s

        public static int kINDEXER_CURRENT_LIMIT = 15;
        public static double kINDEXER_RATIO = 1.0 / 5.0;
        public static double kINDEXER_P = 0.000005;
        public static double kINDEXER_I = 5e-7;
        public static double kINDEXER_D = 0.0;

        public static double kINDEXER_SETPOINT = 5000; // rpm, while moving
        public static double kINDEXER_EJECT_SETPOINT = -1000; // rpm, backwards
    }

    /**Intake Constants */
    public static class IntakeConstants {
        public static double kINTAKE_RATIO = 1.0 / 5.0;
        public static double kINTAKE_SETPOINT = 1000.0; // rpm, while moving
        
        public static int kINTAKE_CURRENT_LIMIT = 15;

        public static double kINTAKE_P = 0.000005;
        public static double kINTAKE_I = 5e-7;
        public static double kINTAKE_D = 0.0;
    }
    /** Climber Constants */
    public static class ClimberConstants {
        public static double kCLIMBER_RATIO = 1.0 / 20.0;
        public static double kCLIMBER_POSITION = 0.0; //rotations
        public static int kCLIMBER_CURRENT_LIMIT = 60; 

        public static double kCLIMBER_P = 0.0;
        public static double kCLIMBER_I = 0.0;
        public static double kCLIMBER_D = 0.0;


    }
    /** Arm Constants */
    public static class ArmConstants {
        public static int kLOWER_RIGHT_PIVOT_CURRENT_LIMIT = 60;
        public static int kLOWER_LEFT_PIVOT_CURRENT_LIMIT = 60;
        public static int kUPPER_PIVOT_CURRENT_LIMIT = 60;
        public static int kELEVATOR_EXT_CURRENT_LIMIT = 60;

        public static double kLOWER_PIVOT_RATIO = 1.0 / 80.0;
        public static double kUPPER_PIVOT_RATIO = 1.0 / 16.0;
        public static double kELEVATOR_EXT_RATIO = 1.0 / 25.0;

        public static double kLOWER_PIVOT_P = 0.5;
        public static double kLOWER_PIVOT_I = 0.0;
        public static double kLOWER_PIVOT_D = 0.2;

        public static double kUPPER_PIVOT_P = 0.0; 
        public static double kUPPER_PIVOT_I = 0.0; 
        public static double kUPPER_PIVOT_D = 0.0; 

        public static double kELEVATOR_EXT_P = 0.0;
        public static double kELEVATOR_EXT_I = 0.0;
        public static double kELEVATOR_EXT_D = 0.0;

        public static double kLOWER_ANGLE_TOLERANCE = 0.0; //deg
        public static double kUPPER_ANGLE_TOLERANCE = 0.0; //deg
        public static double kEXT_TOLERANCE = 0.0; //rots
    }

    /** Swerve drive constants */
    public static class DriveConstants {
        // Max Speeds
        public static double kMAX_TELEOP_TRANSLATIONAL_SPEED = 3.5;
        public static double kMAX_TELEOP_ROTATIONAL_SPEED = 60.0;

        // Base size
        public static Translation2d kSIZE = new Translation2d(14.5, 7.0); // Track Width

        // CANCoder Offsets
        public static double kFL_STEER_ZERO = -0.134277;
        public static double kFR_STEER_ZERO = 0.0;
        public static double kRL_STEER_ZERO = 0.0;
        public static double kRR_STEER_ZERO = 0.0;

        // Pose Estimation
        public static IMUAxis kGYRO_YAW = IMUAxis.kX;
        public static IMUAxis kGYRO_PITCH = IMUAxis.kY;
        public static IMUAxis kGYRO_ROLL = IMUAxis.kZ;
        
        public static Vector<N3> kSTATE_STD_DEV = 
            VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(2.0)); // meters, meters, rads
        public static Vector<N3> kVISION_STD_DEV =
            VecBuilder.fill(0.25, 0.25, Units.degreesToRadians(5.0));

        public static Path kAPRIL_TAG_LAYOUT_JSON = Filesystem.getDeployDirectory().toPath().resolve("AprilTag2024Layout.json");
    }

    /** Swerve module constants */
    public static class SwerveModuleConstants {
        public static final double kWHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI; // meters

        // Drive Motor
        public static final double kDRIVE_RATIO = 6.12; // motor to wheel ratio (L3)
        public static final int kDRIVE_CURRENT_LIMIT = 50;
        public static final double kDRIVE_P = 0.00005;
        public static final double kDRIVE_I = 5e-7;
        public static final double kDRIVE_D = 0.0;
        public static final double kDRIVE_S = 0.0;
        public static final double kDRIVE_V = 0.0;
        public static final double kDRIVE_A = 0.0;

        // Steer Motor
        public static final double kSTEER_RATIO = 150.0 / 7.0;
        public static final int kSTEER_CURRENT_LIMIT = 30;
        public static final double kSTEER_P = 0.45;
        public static final double kSTEER_I = 0.00001;
        public static final double kSTEER_D = 0.0;
        public static final double kSTEER_FF = 0.0;
    }

    /** CAN Hardware Constants */
    public static class HardwareConstants {
        public static int kFL_STEER_CAN = 2;
        public static int kFR_STEER_CAN = 3;
        public static int kRL_STEER_CAN = 4;
        public static int kRR_STEER_CAN = 5;

        public static int kFL_DRIVE_CAN = 6;
        public static int kFR_DRIVE_CAN = 7;
        public static int kRL_DRIVE_CAN = 8;
        public static int kRR_DRIVE_CAN = 9;

        public static int kINTAKE_CAN = 10;
        public static int kINDEXER_CAN = 11;

        public static int kLOWER_LEFT_PIVOT_CAN = 12;
        public static int kLOWER_RIGHT_PIVOT_CAN = 13;
 
        public static int kFL_CANCODER = 14;
        public static int kFR_CANCODER = 15;
        public static int kRL_CANCODER = 16;
        public static int kRR_CANCODER = 17;

        public static int kSHOOTER_UPPER_CAN = 19;
        public static int kSHOOTER_LOWER_CAN = 18;

        public static int kCLIMBER_LEFT_CAN = 20;
        public static int kCLIMBER_RIGHT_CAN = 21; 

        public static int kUPPER_PIVOT_CAN = 22;

        public static int kELEVATOR_EXT_CAN = 23;

        public static String kCANIVORE_BUS = "CANivore";
    }

    public static class LEDConstants {
        public static double kOFF = 0.99;
        public static double kRED = 0.61;
        public static double kBLUE = 0.87;
        public static double kGREEN = 0.77;
        public static double kYELLOW = 0.69;
    }

    
}


