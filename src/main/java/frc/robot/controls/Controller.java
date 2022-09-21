package frc.robot.controls;

import frc.robot.Constants;
import frc.robot.controls.PSController.Axis;
import frc.robot.controls.PSController.Button;
import frc.robot.controls.PSController.Side;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Controller {
    private final double swerveDeadband = Constants.SWERVE_DEADBAND;

    private int mLastDpadLeft = -1;
    private int mLastDpadRight = -1;

    private final int kDpadUp = 0;
    private final int kDpadRight = 90;
    private final int kDpadDown = 180;
    private final int kDpadLeft = 270;

    private static Controller mInstance = null;

    public enum SwerveCardinal {
        NONE(0),

        FORWARDS(0),
        LEFT(270),
        RIGHT(90),
        BACKWARDS(180),

        FAR_FENDER(143),
        RIGHT_FENDER(233),
        LEFT_FENDER(53),
        CLOSE_FENDER(323);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

    public static Controller getInstance() {
        if (mInstance == null) {
            mInstance = new Controller();
        }

        return mInstance;
    }

    public final PSController driver;
    public final PSController operator;

    private Controller() {
        driver = new PSController(Constants.DRIVER_CONTROLLER);
        operator = new PSController(Constants.OPERATOR_CONTROLLER);
    }

    // DRIVER CONTROLS
    public Translation2d getSwerveTranslation() {
        double forwardAxis = driver.getAxis(Side.LEFT, Axis.Y);
        double strafeAxis = driver.getAxis(Side.LEFT, Axis.X);

        forwardAxis = Constants.INVERT_Y_AXIS ? forwardAxis : -forwardAxis;
        strafeAxis = Constants.INVERT_X_AXIS ? strafeAxis :-strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < swerveDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadbandDirection = new Rotation2d(tAxes.getX(), tAxes.getY());
            Translation2d deadbandVector = new Translation2d(swerveDeadband, deadbandDirection);

            double scaled_x = tAxes.getX() - (deadbandVector.getX()) / (1 - deadbandVector.getX());
            double scaled_y = tAxes.getY() - (deadbandVector.getY()) / (1 - deadbandVector.getY());
            return new Translation2d(scaled_x, scaled_y).times(Constants.MAX_DRIVE_SPEED_METERS_PER_SECOND);
        }
    }

    public double getSwerveRotation() {
        double rotAxis = driver.getAxis(Side.RIGHT, Axis.X);
        rotAxis = Constants.INVERT_ANGULAR_AXIS ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < swerveDeadband) {
            return 0.0;
        } else {
            return Constants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * (rotAxis - (Math.signum(rotAxis) * swerveDeadband)) / (1 - swerveDeadband);
        }
    }

    public boolean zeroGyro() {
        return driver.getButton(Button.START) && driver.getButton(Button.BACK);
    }

    public SwerveCardinal getSwerveSnap() {

        // FENDER SNAPS
        if (driver.getButton(Button.A)) {
            return SwerveCardinal.CLOSE_FENDER;
        }
        if (driver.getButton(Button.B)) {
            return SwerveCardinal.LEFT_FENDER;
        }
        if (driver.getButton(Button.X)) {
            return SwerveCardinal.RIGHT_FENDER;
        }
        if (driver.getButton(Button.Y)) {
            return SwerveCardinal.FAR_FENDER;
        }

        // CARDINAL SNAPS

        switch (driver.getController().getPOV()) {
            case kDpadUp:
                return SwerveCardinal.FORWARDS;
            case kDpadLeft:
                return SwerveCardinal.RIGHT;
            case kDpadRight:
                return SwerveCardinal.LEFT;
            case kDpadDown:
                return SwerveCardinal.BACKWARDS;
            default:
                return SwerveCardinal.NONE;
        }
            
    }

    public boolean getBrake() {
        return driver.getButton(Button.LB);
    }

}
