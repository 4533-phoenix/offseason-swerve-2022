package frc.robot.subsystems;

import frc.robot.Constants;
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
        new PIDController(1,0,0), // x
        new PIDController(1,0,0), // y
        new ProfiledPIDController(1,0,0, // rotation
            new TrapezoidProfile.Constraints(6.28,3.14)) // currently max omega = 1 rot/s and max alpha = 180 deg/s^2
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
