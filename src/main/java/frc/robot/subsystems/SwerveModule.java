package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final PIDController steerPIDController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorID, int steerMotorID, boolean driveMotorReversed, boolean steerMotorReversed,
            int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderID);
        
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        steerMotor = new CANSparkMax(steerMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_METERS_PER_ROTATION);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_METERS_PER_SECOND);
        steerEncoder.setPositionConversionFactor(ModuleConstants.STEER_ENCODER_RADIANS_PER_ROTATION);
        steerEncoder.setVelocityConversionFactor(ModuleConstants.STEER_ENCODER_RADIANS_PER_SECOND);

        steerPIDController = new PIDController(ModuleConstants.STEER_KP, 0, 0);
        steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getSteerPosition() {
        return steerEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getSteerVelocity() {
        return steerEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition();
        angle *= Math.PI / 180.0;
        angle %= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;

        if (angle < 0.0)
            angle += 2.0 * Math.PI;

        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public double getRawAbsolutePosition() {
        return absoluteEncoder.getPosition();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        steerEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.DRIVE_MAX_SPEED_METERS_PER_SECOND);
        steerMotor.set(steerPIDController.calculate(getSteerPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }
}
