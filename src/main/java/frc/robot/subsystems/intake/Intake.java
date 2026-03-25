package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeIOConstants.INTAKE_DOWN_ANGLE;
import static frc.robot.subsystems.intake.IntakeIOConstants.INTAKE_UP_ANGLE;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeIO io;

  // private LinearSystem<N1, N1, N1> intakeArm;
  // private SysIdRoutine sysid;

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
    if (setpoint >= ((Math.abs(INTAKE_UP_ANGLE - INTAKE_DOWN_ANGLE)) / 2)) {
      io.turnIntakeMotorAngle(INTAKE_UP_ANGLE);
    } else {
      io.turnIntakeMotorAngle(INTAKE_DOWN_ANGLE);
    }
  }

  public void incrementIntake() {
    setpoint += 2;
    io.turnIntakeMotorAngle(setpoint);
  }

  public void decrementIntake() {
    setpoint -= 2;
    io.turnIntakeMotorAngle(setpoint);
  }
}
