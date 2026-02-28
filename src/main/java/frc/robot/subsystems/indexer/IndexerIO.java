package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    boolean indexerMotorConnected = false;
    double indexerMotorSpeedRadsPerSecond = 0.0;
    double indexerMotorAppliedVolts = 0.0;
    double indexerMotorCurrentAmps = 0.0;
  }

  public default void updateInputs(IndexerIOInputs inputs) {}

  public default void setIndexerSpeed(double speed) {}
}
