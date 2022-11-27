// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.loops.Looper;
import frc.robot.subsystems.Swerve;
import frc.robot.controls.Controller;
import frc.robot.logger.*;

import frc.robot.controls.Controller.SwerveCardinal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Instantiate enabled and disabled loopers

  private final Looper mEnabledLooper = new Looper();
  private final Looper mDisabledLooper = new Looper();

  // Instantiate logging looper
  private final Looper mLoggingLooper = new Looper();

  // Subsystem instances

  private final Controller mController = Controller.getInstance();

  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  private final Swerve mSwerve = Swerve.getInstance();

  private LoggingSystem mLogger = LoggingSystem.getInstance();

  @Override
  public void robotInit() {
    
    try {

      mSubsystemManager.setSubsystems(
        mSwerve
      );

      mSubsystemManager.registerEnabledLoops(mEnabledLooper);
      mSubsystemManager.registerDisabledLoops(mDisabledLooper);

      mSubsystemManager.registerLoggingSystems(mLogger);
      mLogger.registerLoops(mLoggingLooper);

    } catch (Throwable t) {
      throw t;
    }

  }

  @Override
  public void robotPeriodic() {
    mEnabledLooper.outputToSmartDashboard();
    mSwerve.printModuleOffsets();
  }

  @Override
  public void autonomousInit() {

    try {
      mDisabledLooper.stop();
			mEnabledLooper.start();
      mLoggingLooper.start();
    } catch (Throwable t) {
      throw t;
    }

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {

    try {
      mDisabledLooper.stop();
			mEnabledLooper.start();
      mLoggingLooper.start();
    } catch (Throwable t) {
      throw t;
    }

  }

  @Override
  public void teleopPeriodic() {

    if (mController.zeroGyro()) {
      mSwerve.zeroHeading();
    }

    Translation2d swerveTranslation = new Translation2d(mController.getSwerveTranslation().getX(),
        mController.getSwerveTranslation().getY());
    double swerveRotation = mController.getSwerveRotation();

    mSwerve.drive(swerveTranslation, swerveRotation, true, true);
    
  }

  @Override
  public void disabledInit() {
    try {
      mEnabledLooper.stop();
			mDisabledLooper.start();
      mLoggingLooper.stop();
    } catch (Throwable t) {
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    try {
      mDisabledLooper.outputToSmartDashboard();
    } catch (Throwable t) {
      throw t;
    }

  }

  @Override
  public void testInit() {
    try {
      mEnabledLooper.stop();
			mDisabledLooper.stop();
      mLoggingLooper.stop();
    } catch (Throwable t) {

      throw t;

    }

  }

  @Override
  public void testPeriodic() {

  }
}
