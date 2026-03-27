package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    boolean intakeMotorConnected = false;
    double setpoint = 0.0;
    double intakeMotorSpeedRPM = 0.0;
    double intakeMotorAppliedVolts = 0.0;
    double intakeMotorCurrentAmps = 0.0;

    double turnIntakeAbsoluteRotations = 0.0;
    double turnIntakeSetpoint = 0.0;

    boolean turnLeftIntakeMotorConnected = false;
    double turnLeftIntakeMotorAppliedVolts = 0.0;
    double turnLeftIntakeMotorCurrentAmps = 0.0;

    boolean turnRightIntakeMotorConnected = false;
    double turnRightIntakeMotorAppliedVolts = 0.0;
    double turnRightIntakeMotorCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeSpeed(double speed) {}

  public default void turnIntakeMotorAngle(double degrees) {}
}
