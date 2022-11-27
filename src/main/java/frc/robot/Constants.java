package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

	public static final class GlobalConstants {
		// Robot loop time
		public static final double LOOPER_TIME = 0.02;
	}

    public static final class AutoConstants {
        // Holonomic controller PID constants - x
        public static final double AUTO_X_KP = 1.0;
        public static final double AUTO_X_KI = 0.0;
        public static final double AUTO_X_KD = 0.0;

        // Holonomic controller PID constants - y
        public static final double AUTO_Y_KP = 1.0;
        public static final double AUTO_Y_KI = 0.0;
        public static final double AUTO_Y_KD = 0.0;

        // Holonomic controller PID constants - theta
        public static final double AUTO_THETA_KP = 1.0;
        public static final double AUTO_THETA_KI = 0.0;
        public static final double AUTO_THETA_KD = 0.0;
        // Max angular velocity, acceleration in rad/s
        public static final TrapezoidProfile.Constraints AUTO_THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(6.28,3.14);
    }

	public static final class ModuleConstants {
        public static final double WHEEL_DIAMETER_INCHES = 4;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);

        public static final double DRIVE_GEAR_RATIO = 1.0 / 6.75;
        public static final double STEER_GEAR_RATIO = 1.0 / 12.8;

        public static final double DRIVE_ENCODER_METERS_PER_ROTATION = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double STEER_ENCODER_RADIANS_PER_ROTATION = STEER_GEAR_RATIO * 2.0 * Math.PI;
        public static final double DRIVE_ENCODER_METERS_PER_SECOND = DRIVE_ENCODER_METERS_PER_ROTATION / 60.0;
        public static final double STEER_ENCODER_RADIANS_PER_SECOND = STEER_ENCODER_RADIANS_PER_ROTATION / 60.0;
        
        public static final double STEER_KP = 0.5;
    }

    public static final class DriveConstants {
		// Left-to-right distance between wheel centers in meters
        public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
		// Front-to-back distance between wheel centers in meters
        public static final double WHEEL_BASE = Units.inchesToMeters(22.5);
        
		// Creates kinematics object for swerve subsystem
        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 8;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 2;
        public static final int BACK_LEFT_DRIVE_MOTOR_ID = 4;
        public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 6;

        public static final int FRONT_LEFT_STEER_MOTOR_ID = 1;
        public static final int FRONT_RIGHT_STEER_MOTOR_ID = 3;
        public static final int BACK_LEFT_STEER_MOTOR_ID = 5;
        public static final int BACK_RIGHT_STEER_MOTOR_ID = 7;

        public static final boolean FRONT_LEFT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_DRIVE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_DRIVE_ENCODER_REVERSED = false;

        public static final boolean FRONT_LEFT_STEER_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_STEER_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_STEER_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_STEER_ENCODER_REVERSED = false;

        public static final int FRONT_LEFT_STEER_ABSOLUTE_ENCODER_ID = 9;
        public static final int FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 10;
        public static final int BACK_LEFT_STEER_ABSOLUTE_ENCODER_ID = 11;
        public static final int BACK_RIGHT_STEER_ABSOLUTE_ENCODER_ID = 12;

        public static final boolean FRONT_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_LEFT_STEER_ABSOLUTE_ENCODER_REVERSED = false;
        public static final boolean BACK_RIGHT_STEER_ABSOLUTE_ENCODER_REVERSED = false;

		// Offsets for swerve modules. Should be equal to absolute encoder reading when wheel is facing straight forward
        public static final double FRONT_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET = 0.6105;
        public static final double FRONT_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET = 4.3657;
        public static final double BACK_LEFT_STEER_ABSOLUTE_ENCODER_OFFSET = 6.1605;
        public static final double BACK_RIGHT_STEER_ABSOLUTE_ENCODER_OFFSET = 2.9207;

        public static final double DRIVE_MAX_SPEED_METERS_PER_SECOND = 3.0;
        public static final double STEER_MAX_SPEED_RADIANS_PER_SECOND = 2.0 * 2.0 * Math.PI;

        public static final double TELEOP_DRIVE_MAX_ACCELERATION_UNITS_PER_SECOND = 5.0;
        public static final double TELEOP_MAX_ANGULAR_ACCELERATION_UNITS_PER_SECOND = 3.0;
    }

	public static final class OIConstants {
		public static final int DRIVER_CONTROLLER_PORT = 0;

		// Controller axis IDs
        public static final int DRIVER_Y_AXIS = 1;
        public static final int DRIVER_X_AXIS = 0;
        public static final int DRIVER_ROT_AXIS = 4;
        
		// Deadband for drive axes
        public static final double DRIVE_DEADBAND = 0.05;

		// Threshold for trigger being pressed
		public static final double TRIGGER_THRESHOLD = 0.2;
	}
}
