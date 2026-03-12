package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeIOConstants.INTAKE_DOWN_ANGLE;
import static frc.robot.subsystems.intake.IntakeIOConstants.INTAKE_UP_ANGLE;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeIO io;

  private boolean intakeToggle = true;
  private double setpoint;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void runIntake(double speed) {
    io.setIntakeSpeed(speed);
  }

  public void stopIntake() {
    io.setIntakeSpeed(0);
  }

  public void toggleIntake() {
    if (!intakeToggle) {
      io.turnIntakeMotorAngle(INTAKE_DOWN_ANGLE);
    } else {
      io.turnIntakeMotorAngle(INTAKE_UP_ANGLE);
    }
    intakeToggle = !intakeToggle;
  }

  public void incrementIntake() {
    io.turnIntakeMotorAngle(++setpoint);
  }

  public void decrementIntake() {
    io.turnIntakeMotorAngle(--setpoint);
  }
}
