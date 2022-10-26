package frc.robot.subsystems;

// Credit: Swerve Drive Specialties, Team 1678

import frc.robot.Constants;
import frc.robot.loops.*;
import frc.robot.logger.*;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import java.util.*;

public class Swerve extends Subsystem {
    
    private static Swerve mInstance;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    LogStorage<PeriodicIO> mStorage = null;

    public boolean isEnabled = false;
    public boolean isSnapping;

    // Max voltage that can be delivered to the drive motors. Use to cap max speed.
    public static final double MAX_VOLTAGE = 12.0;

    // Theoretical max robot velocity for moving in a straight line.
    // Formula: <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    public static final double  MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0 *
        SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

    // Max robot angular velocity in radians per second.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    public ProfiledPIDController snapPIDController;

    private boolean mLocked = false;

    public boolean getLocked() {
        return mLocked;
    }

    public void setLocked(boolean lock) {
        mLocked = lock;
    }

    public SwerveModule[] swerveMods;

    public final SwerveModule frontRightModule; 
    public final SwerveModule frontLeftModule;
    public final SwerveModule backRightModule;
    public final SwerveModule backLeftModule;

    // Object representing the robot's speed (vx, vy, angular velocity)
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    }

    public Swerve() {
        frontRightModule = Mk4SwerveModuleHelper.createNeo(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.DRIVE_RF,
            Constants.STEER_RF,
            Constants.STEER_RF_ENCODER,
            Constants.RF_STEER_OFFSET
        );

        frontLeftModule = Mk4SwerveModuleHelper.createNeo(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.DRIVE_LF,
            Constants.STEER_LF,
            Constants.STEER_LF_ENCODER,
            Constants.LF_STEER_OFFSET
        );

        backRightModule = Mk4SwerveModuleHelper.createNeo(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.DRIVE_RB,
            Constants.STEER_RB,
            Constants.STEER_RB_ENCODER,
            Constants.RB_STEER_OFFSET
        );

        backLeftModule = Mk4SwerveModuleHelper.createNeo(
            Mk4SwerveModuleHelper.GearRatio.L2,
            Constants.DRIVE_LB,
            Constants.STEER_LB,
            Constants.STEER_LB_ENCODER,
            Constants.LB_STEER_OFFSET
        );

        swerveMods = new SwerveModule[] {
            frontRightModule,
            frontLeftModule,
            backRightModule,
            backLeftModule
        };

        snapPIDController = new ProfiledPIDController(
            Constants.SNAP_KP,
            Constants.SNAP_KI, 
            Constants.SNAP_KD, 
            Constants.SNAP_KTHETA_CONTROLLER_CONSTRAINTS);
        snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

        zeroGyroscope();
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                isEnabled = true;
            }

            @Override
            public void onLoop(double timestamp) {
                isEnabled = false;
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        if (isSnapping) {
            if (Math.abs(rotation) == 0.0) {
                maybeStopSnap(false);
                rotation = calculateSnapValue();
            } else {
                maybeStopSnap(true);
            }
        }

        SwerveModuleState[] swerveModuleStates = null;
        if (mLocked) {
            swerveModuleStates = new SwerveModuleState[]{
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
                new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
            };
        } else {
            swerveModuleStates = kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    new Rotation2d(navX.getAngle())
                ) : new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation
                )
            );
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_DRIVE_SPEED_METERS_PER_SECOND);

        for (int mod = 0; mod < swerveMods.length; mod++) {
            swerveMods[mod].set(swerveModuleStates[mod].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, 
                    swerveModuleStates[mod].angle.getRadians());
        }
    }

    public double calculateSnapValue() {
        return snapPIDController.calculate(new Rotation2d(navX.getAngle()).getRadians());
    }

    public void startSnap(double snapAngle) {
        snapPIDController.reset(new Rotation2d(navX.getAngle()).getRadians());
        snapPIDController.setGoal(new TrapezoidProfile.State(Math.toRadians(snapAngle), 0.0));
        isSnapping = true;
    }

    private boolean snapComplete() {
        double error = snapPIDController.getGoal().position = new Rotation2d(navX.getAngle()).getRadians();
        return Math.abs(error) < Math.toRadians(1.0); // Error should be less than 1 degree. Probably should implement a timer here
    }

    public void maybeStopSnap(boolean force) {
        if (!isSnapping) {
            return;
        }
        if (force || snapComplete()) {
            isSnapping = false;
            snapPIDController.reset(new Rotation2d(navX.getAngle()).getRadians());
        }
    }

    public void zeroGyroscope() {
        navX.zeroYaw();
    }

    public Rotation2d getGyroscopeRotation() {   
       if (navX.isMagnetometerCalibrated()) {
         // We will only get valid fused headings if the magnetometer is calibrated
         return Rotation2d.fromDegrees(navX.getFusedHeading());
       }
    
       // Turning counter-clockwise should make the angle increase
       return Rotation2d.fromDegrees(360.0 - navX.getYaw());
      }

      public void drive(ChassisSpeeds speeds) {
        chassisSpeeds = speeds;
      }

      @Override
      public void stop() {
          isEnabled = true;
      }

      @Override
      public boolean checkSystem() {
          return true;
      }

      @Override
      public void readPeriodicInputs() {
          mPeriodicIO.gyro_heading = new Rotation2d(navX.getYaw()).getDegrees();
          mPeriodicIO.robot_pitch = new Rotation2d(navX.getPitch()).getDegrees();
          mPeriodicIO.robot_roll = new Rotation2d(navX.getRoll()).getDegrees();
          mPeriodicIO.snap_target = Math.toDegrees(snapPIDController.getGoal().position);
          mPeriodicIO.swerve_heading = MathUtil.inputModulus(new Rotation2d(navX.getYaw()).getDegrees(), 0, 360);
  
          SendLog();
      }
  
      public static class PeriodicIO {
          // inputs
          public double gyro_heading;
          public double robot_pitch;
          public double robot_roll;
          public double swerve_heading;
  
          public double angular_velocity;
          public double goal_velocity;
  
          // outputs
          public double snap_target;
      }
  
      //logger
      @Override
      public void registerLogger(LoggingSystem LS) {
          SetupLog();
          LS.register(mStorage, "SWERVE_LOGS.csv");
      }
      
      public void SetupLog() {
          mStorage = new LogStorage<PeriodicIO>();
  
          ArrayList<String> headers = new ArrayList<String>();
          headers.add("timestamp");
          headers.add("is_enabled");
          headers.add("gyro_heading");
          headers.add("robot_pitch");
          headers.add("robot_roll");
          headers.add("snap_target");
          headers.add("swerve_heading");
  
          mStorage.setHeaders(headers);
      }
  
      public void SendLog() {
          ArrayList<Number> items = new ArrayList<Number>();
          items.add(isEnabled ? 1.0 : 0.0);
          items.add(mPeriodicIO.gyro_heading);
          items.add(mPeriodicIO.robot_pitch);
          items.add(mPeriodicIO.robot_roll);
          items.add(mPeriodicIO.snap_target);
          items.add(mPeriodicIO.swerve_heading);
      }
}
