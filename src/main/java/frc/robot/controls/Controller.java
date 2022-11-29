package frc.robot.controls;

import frc.robot.Constants.*;
import frc.robot.controls.PSController.Axis;
import frc.robot.controls.PSController.Button;
import frc.robot.controls.PSController.Side;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Controller {
    private final double swerveDeadband = OIConstants.DRIVE_DEADBAND;

    private static Controller mInstance = null;

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
        double forwardAxis = Math.pow(driver.getAxis(Side.LEFT, Axis.Y),3) * DriveConstants.DRIVE_MAX_SPEED_METERS_PER_SECOND;
        double strafeAxis = Math.pow(driver.getAxis(Side.LEFT, Axis.X),3) * DriveConstants.DRIVE_MAX_SPEED_METERS_PER_SECOND;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < swerveDeadband) {
            return new Translation2d();
        } else {
            return tAxes;
        }
    }

    public double getSwerveRotation() {
        double rotAxis = (driver.getAxis(Side.RIGHT, Axis.X) / Math.abs(driver.getAxis(Side.RIGHT, Axis.X)) * Math.pow(driver.getAxis(Side.RIGHT, Axis.X),2));

        if (Math.abs(rotAxis) < swerveDeadband) {
            return 0.0;
        } else {
            return rotAxis;
        }
    }

    public boolean zeroGyro() {
        return driver.getButton(Button.START);
    }
}
