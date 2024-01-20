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
    /** Swerve drive constants */
    public static class DriveConstants {
        // Base size
        public static Translation2d kSIZE = new Translation2d(14.5, 7.0); // Track Width

        // CANCoder Offsets
        public static double kFL_STEER_ZERO = 0.0;
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
        public static final double kDRIVE_STAGE_1_RATIO = 50.0 / 14.0; // motor to base pulley ratio
        public static final double kDRIVE_STAGE_2_RATIO = (16.0 / 28.0) * (45.0 / 15.0); // base pulley to wheel ratio
        public static final double kDRIVE_PEAK_VOLTAGE = 16;
        public static final double kDRIVE_STATOR_CURRENT_LIMIT = 60;
        public static final double kDRIVE_SUPPLY_CURRENT_LIMIT = 60;
        public static final double kDRIVE_P = 0.1;
        public static final double kDRIVE_I = 0.0;
        public static final double kDRIVE_D = 0.0;
        public static final double kDRIVE_S = 0.0;
        public static final double kDRIVE_V = 0.0;
        public static final double kDRIVE_A = 0.0;
        public static final double kDRIVE_MAX_VELO = 160; // r/s
        public static final double kDRIVE_MAX_ACCEL = 160; // r/s^2

        // Spin Motor
        public static final double kSTEER_RATIO = 150.0 / 7.0;
        public static final int kSTEER_CURRENT_LIMIT = 30;
        public static final double kSTEER_P = 0.1;
        public static final double kSTEER_I = 0.0;
        public static final double kSTEER_D = 0.0;
    }

    /** CAN Hardware Constants */
    public static class HardwareConstants {
        public static int kFL_DRIVE_CAN = 3;
        public static int kFR_DRIVE_CAN = 4;
        public static int kRL_DRIVE_CAN = 5;
        public static int kRR_DRIVE_CAN = 6;
        public static int kFL_STEER_CAN = 7;
        public static int kFR_STEER_CAN = 8;
        public static int kRL_STEER_CAN = 9;
        public static int kRR_STEER_CAN = 10;
        public static int kFL_CANCODER = 11;
        public static int kFR_CANCODER = 12;
        public static int kRL_CANCODER = 13;
        public static int kRR_CANCODER = 14;

        public static String kCANIVORE_BUS = "Canivore";
    }

    public static class LEDConstants {
        public static double kOFF = 0.99;
        public static double kRED = 0.61;
        public static double kBLUE = 0.87;
        public static double kGREEN = 0.77;
        public static double kYELLOW = 0.69;
    }

    public static class IntakeConstants {
        public static int = ;
    }
}


