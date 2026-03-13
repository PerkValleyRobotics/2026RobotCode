package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    boolean hopperMotorConnected = false;
    double hopperMotorSpeedRadsPerSecond = 0.0;
    double hopperMotorAppliedVolts = 0.0;
    double hopperMotorCurrentAmps = 0.0;

    boolean indexerMotorConneced = false;
    double indexerMotorSpeedRadsPerSecond = 0.0;
    double indexerMotorAppliedVolts = 0.0;
    double indexerMotorCurrentAmps = 0.0;
  }

  public default void updateInputs(HopperIOInputs inputs) {}

  public default void setHopperSpeed(double speed) {}

  public default void setIndexerSpeed(double speed) {}

  public default boolean motorIsOverTemp() {
    return false;
  }
}
