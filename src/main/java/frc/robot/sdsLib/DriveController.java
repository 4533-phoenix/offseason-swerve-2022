package frc.robot.sdsLib;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
