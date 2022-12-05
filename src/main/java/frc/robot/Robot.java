// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.loops.Looper;
import frc.robot.subsystems.Swerve;
import frc.robot.controls.DriveController;
import frc.robot.controls.PSController.Button;
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

  // Controller instance
  private final DriveController mController = DriveController.getInstance();

  // Subsystem instances
  private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();
  private final Swerve mSwerve = Swerve.getInstance();

  @Override
  public void robotInit() {
    mSubsystemManager.setSubsystems(
      mSwerve
    );

    mSubsystemManager.registerEnabledLoops(mEnabledLooper);
    mSubsystemManager.registerDisabledLoops(mDisabledLooper);
  }

  @Override
  public void robotPeriodic() {
    mEnabledLooper.loop();
  }

  @Override
  public void autonomousInit() {
    mDisabledLooper.stop();
    mEnabledLooper.start();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    mDisabledLooper.stop();
    mEnabledLooper.start();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    mEnabledLooper.stop();
    mDisabledLooper.start();
  }

  @Override
  public void disabledPeriodic() {
    mDisabledLooper.loop();
  }

  @Override
  public void testInit() {
    mEnabledLooper.stop();
    mDisabledLooper.stop();
  }

  @Override
  public void testPeriodic() {}
}
