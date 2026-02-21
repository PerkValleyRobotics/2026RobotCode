package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
  @AutoLog
  public static class LauncherIOInputs {
    // shooting data
    boolean shootingMotorConnected = false;
    double shootingMotorSpeedRadsPerSecond = 0.0;
    double shootingMotorAppliedVolts = 0.0;
    double shootingMotorCurrentAmps = 0.0;

    // turn motor data
    boolean turningMotorConnected = false;
    double turningMotorPositionDeg = 0.0;
    double turningMotorAppliedVolts = 0.0;
    double turningMotorCurrentAmps = 0.0;
  }

  public default void updateInputs(LauncherIOInputs inputs) {}

  public default void setShooterSpeed(double speed) {}

  public default void turnHoodAngle(double angleDegrees) {}
}
