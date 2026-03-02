package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    boolean intakeMotorConnected = false;
    double intakeMotorSpeedRadsPerSecond = 0.0;
    double intakeMotorAppliedVolts = 0.0;
    double intakeMotorCurrentAmps = 0.0;

    boolean turningIntakeMotorConnected = false;
    double turningIntakeMotorPositionDeg = 0.0;
    double turningIntakeMotorAppliedVolts = 0.0;
    double turningIntakeMotorCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeSpeed(double speed) {}

  public default void turnIntakeMotorAngle(double degrees) {}
}
