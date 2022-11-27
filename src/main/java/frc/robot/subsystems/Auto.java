package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.loops.*;
import frc.robot.logger.*;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class Auto extends Subsystem {
    private static Auto mInstance;

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    LogStorage<PeriodicIO> mStorage = null;

    public boolean isEnabled = false;

    HolonomicDriveController autoController = new HolonomicDriveController(
        new PIDController(
            AutoConstants.AUTO_X_KP,
            AutoConstants.AUTO_X_KI,
            AutoConstants.AUTO_X_KD
        ),
        new PIDController(
            AutoConstants.AUTO_Y_KP,
            AutoConstants.AUTO_Y_KI,
            AutoConstants.AUTO_Y_KD
        ),
        new ProfiledPIDController(
            AutoConstants.AUTO_THETA_KP,
            AutoConstants.AUTO_THETA_KI,
            AutoConstants.AUTO_THETA_KD,
            AutoConstants.AUTO_THETA_CONSTRAINTS
        )
    );

    public static Auto getInstance() {
        if (mInstance == null) {
            mInstance = new Auto();
        }
        return mInstance;
    }

    @Override
    public void stop() {
        isEnabled = true;
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public static class PeriodicIO {

    }
}
