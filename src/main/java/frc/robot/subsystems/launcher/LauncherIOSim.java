package frc.robot.subsystems.launcher;

import static frc.robot.subsystems.launcher.LauncherConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class LauncherIOSim implements LauncherIO {
  private final DCMotorSim shootingMotor;
  private final DCMotorSim turningMotor;
  private PIDController turnController = new PIDController(TURN_MOTOR_SIM_kP, 0, TURN_MOTOR_SIM_kD);
  private final boolean setTurningOpenLoop = false;

  private double turningFFVolts = 0.0;
  private double shooterAppliedVolts = 0.0;
  private double turningAppliedVolts = 0.0;

  public LauncherIOSim() {
    shootingMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(shootingSimMotor, 1, 1), shootingSimMotor);
    turningMotor =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(turningSimMotor, 1, 1), turningSimMotor);
    turnController.enableContinuousInput(HOOD_ANGLE_MIN_LIMIT, HOOD_ANGLE_MAX_LIMIT);
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    if (!setTurningOpenLoop) {
      turningAppliedVolts =
          turningFFVolts + turnController.calculate(turningMotor.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Update simulation state
    shootingMotor.setInputVoltage(MathUtil.clamp(shooterAppliedVolts, -12.0, 12.0));
    turningMotor.setInputVoltage(MathUtil.clamp(turningAppliedVolts, -12.0, 12.0));
    shootingMotor.update(0.02);
    turningMotor.update(0.02);

    // Update shooting motor
    inputs.shootingMotorConnected = true;
    inputs.shootingMotorSpeedRadsPerSecond = shootingMotor.getAngularVelocityRadPerSec();
    inputs.shootingMotorAppliedVolts = shooterAppliedVolts;
    inputs.shootingMotorCurrentAmps = Math.abs(shootingMotor.getCurrentDrawAmps());

    // Update motor inputs
    inputs.turningMotorConnected = true;
    inputs.turningMotorAppliedVolts = turningAppliedVolts;
    inputs.turningMotorCurrentAmps = Math.abs(turningMotor.getCurrentDrawAmps());
  }

  @Override
  public void setShooterSpeed(double speed) {}

  public void turnHoodAngle(double angleDegrees) {}
}
