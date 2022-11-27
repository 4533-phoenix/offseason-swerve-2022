package frc.robot.controls;

import frc.robot.Constants.*;
import frc.robot.controls.PSController.Axis;
import frc.robot.controls.PSController.Button;
import frc.robot.controls.PSController.Side;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Controller {
    private final double swerveDeadband = OIConstants.DRIVE_DEADBAND;

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

        FAR_FENDER(135),
        RIGHT_FENDER(225),
        LEFT_FENDER(45),
        CLOSE_FENDER(315);

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

    private Controller() {
        driver = new PSController(OIConstants.DRIVER_CONTROLLER_PORT);
    }

    // DRIVER CONTROLS
    public Translation2d getSwerveTranslation() {
        double forwardAxis = driver.getAxis(Side.LEFT, Axis.Y);
        double strafeAxis = driver.getAxis(Side.LEFT, Axis.X);

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < swerveDeadband) {
            return new Translation2d();
        } else {
            return tAxes;
        }
    }

    public double getSwerveRotation() {
        double rotAxis = driver.getAxis(Side.RIGHT, Axis.X);

        if (Math.abs(rotAxis) < swerveDeadband) {
            return 0.0;
        } else {
            return rotAxis;
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
