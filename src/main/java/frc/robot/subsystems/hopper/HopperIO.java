package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    boolean hopperMotorConnected = false;
    double hopperMotorSpeedRadsPerSecond = 0.0;
    double hopperMotorAppliedVolts = 0.0;
    double hopperMotorCurrentAmps = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setHopperSpeed(double speed) {}
}
