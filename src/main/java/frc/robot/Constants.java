package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {

    // Robot loop time
    public static final double kLooperDt = 0.02;

    // Drive motor IDs

    public static final int DRIVE_RF = 2;
	public static final int STEER_RF = 3;
	public static final int DRIVE_LF = 8;
	public static final int STEER_LF = 1;
    public static final int DRIVE_RB = 6;
	public static final int STEER_RB = 7;
    public static final int DRIVE_LB = 4;
	public static final int STEER_LB = 5;

	// Drive encoder IDs

	public static final int STEER_RF_ENCODER = 1;
	public static final int STEER_LF_ENCODER = 0;
	public static final int STEER_RB_ENCODER = 3;
	public static final int STEER_LB_ENCODER = 2;

	// Drive controller IDs

	public static final int DRIVER_CONTROLLER = 0;
	public static final int OPERATOR_CONTROLLER = 1;

	// Controller button IDs

	public static final int BUTTON_A = 1;
	public static final int BUTTON_B = 2;
	public static final int BUTTON_X = 3;
	public static final int BUTTON_Y = 4;
	public static final int BUTTON_LB = 5;
	public static final int BUTTON_RB = 6;
	public static final int BUTTON_BACK = 7;
	public static final int BUTTON_START = 8;
	public static final int LEFT_STICK_PRESS_DOWN = 9;
	public static final int RIGHT_STICK_PRESS_DOWN = 10;

	// Drive controller axis IDs

	public static final int LEFT_STICK_AXIS = 1;
	public static final int RIGHT_STICK_AXIS = 5;
	public static final int LEFT_TRIGGER_AXIS = 2;
	public static final int RIGHT_TRIGGER_AXIS = 3;

	public static final double TRIGGER_THRESHOLD = 0.2;

	// Swerve constants
	// TODO: put in measurements from CAD

	public static final boolean INVERT_Y_AXIS = false;
	public static final boolean INVERT_X_AXIS = false;
	public static final boolean INVERT_ANGULAR_AXIS = false;

	public static final double SWERVE_DEADBAND = 0.15;

	// Left-to-right distance between drivetrain wheels (center-to-center)
	public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5715;

	// Front-to-back distance between drivetrain wheels (center-to-center)
	public static final double DRIVETRAIN_WHEELBASE_METERS = 0.5715;

	// Offsets for each steer motor in radians (ideally straight forward should be set to zero)
	public static final double RF_STEER_OFFSET = 0.0974;
	public static final double LF_STEER_OFFSET = 0.3525;
	public static final double RB_STEER_OFFSET = 1.0;
	public static final double LB_STEER_OFFSET = 0.7535;

	public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2.0 * Math.PI;

	public static final double MAX_DRIVE_SPEED_METERS_PER_SECOND = 4.5;

	// Snap PID constants - to be measured
	public static final double SNAP_KP = 0.0;
	public static final double SNAP_KI = 0.0;
	public static final double SNAP_KD = 0.0;
	public static final TrapezoidProfile.Constraints SNAP_KTHETA_CONTROLLER_CONSTRAINTS = 
		new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, Math.pow(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, 2));
}
