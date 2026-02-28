package frc.robot.subsystems.launcher;

import static frc.robot.subsystems.launcher.LauncherConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class LauncherIOSpark implements LauncherIO {
  private final SparkBase shootingMotor;
  private final SparkBase turningMotor;
  private final RelativeEncoder shootingEncoder;
  private final RelativeEncoder turningEncoder;

  private final Debouncer shootDebounce = new Debouncer(0.05);
  private final Debouncer turnDebounce = new Debouncer(0.05);

  private SparkClosedLoopController turningController;

  public LauncherIOSpark() {
    shootingMotor = new SparkFlex(SHOOTING_MOTOR_ID, MotorType.kBrushless);
    turningMotor = new SparkMax(TURNING_MOTOR_ID, MotorType.kBrushless);

    shootingEncoder = shootingMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    turningController = turningMotor.getClosedLoopController();

    SparkBaseConfig shooterConf = new SparkFlexConfig();
    SparkBaseConfig turnConf = new SparkMaxConfig();

    shooterConf
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(SHOOTING_MOTOR_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(false);
    shooterConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    tryUntilOk(
        shootingMotor,
        5,
        () ->
            shootingMotor.configure(
                shooterConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(shootingMotor, 5, () -> shootingEncoder.setPosition(0));

    turnConf
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TURNING_MOTOR_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(false);
    turnConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    turnConf
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(TURN_MOTOR_kP)
        .i(TURN_MOTOR_kI)
        .d(TURN_MOTOR_kD);

    tryUntilOk(
        turningMotor,
        5,
        () ->
            turningMotor.configure(
                turnConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(turningMotor, 5, () -> turningEncoder.setPosition(0));
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    boolean sparkStickyFault = false;

    inputs.shootingMotorConnected = shootDebounce.calculate(sparkStickyFault);

    ifOk(
        shootingMotor,
        shootingEncoder::getVelocity,
        (value) -> inputs.shootingMotorSpeedRadsPerSecond = value);
    ifOk(
        shootingMotor,
        new DoubleSupplier[] {shootingMotor::getBusVoltage, shootingMotor::getAppliedOutput},
        (value) -> inputs.shootingMotorAppliedVolts = value[0] * value[1]);
    ifOk(
        shootingMotor,
        shootingMotor::getOutputCurrent,
        (value) -> inputs.shootingMotorCurrentAmps = value);

    inputs.turningMotorConnected = turnDebounce.calculate(sparkStickyFault);

    ifOk(
        turningMotor,
        turningEncoder::getPosition,
        (value) -> inputs.turningMotorPositionDeg = value);
    ifOk(
        turningMotor,
        new DoubleSupplier[] {turningMotor::getBusVoltage, turningMotor::getAppliedOutput},
        (value) -> inputs.turningMotorAppliedVolts = value[0] * value[1]);
    ifOk(
        turningMotor,
        turningMotor::getOutputCurrent,
        (value) -> inputs.turningMotorCurrentAmps = value);
  }

  @Override
  public void setShooterSpeed(double speed) {
    shootingMotor.set(speed);
  }

  public void turnHoodAngle(double angleDegrees) {
    turningController.setSetpoint(angleDegrees, ControlType.kPosition);
  }
}
