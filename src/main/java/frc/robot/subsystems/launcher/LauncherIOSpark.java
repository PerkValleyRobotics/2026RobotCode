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
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import java.util.function.DoubleSupplier;

public class LauncherIOSpark implements LauncherIO {
  private final SparkBase leftShootingMotor;
  private final SparkBase rightShootingMotor;

  private final SparkBase turningMotor;
  private final RelativeEncoder shootingEncoder;
  private final RelativeEncoder turningEncoder;

  private final Debouncer shootDebounce = new Debouncer(0.05);
  private final Debouncer turnDebounce = new Debouncer(0.05);

  private SparkClosedLoopController turningController;

  public LauncherIOSpark() {
    leftShootingMotor = new SparkFlex(LEFT_SHOOTING_MOTOR_ID, MotorType.kBrushless);
    rightShootingMotor = new SparkFlex(RIGHT_SHOOTING_MOTOR_ID, MotorType.kBrushless);
    turningMotor = new SparkFlex(TURNING_MOTOR_ID, MotorType.kBrushless);

    shootingEncoder = leftShootingMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    turningController = turningMotor.getClosedLoopController();

    SparkBaseConfig shooterConf = new SparkFlexConfig();
    SparkBaseConfig followerConf = new SparkFlexConfig();
    SparkBaseConfig turnConf = new SparkMaxConfig();

    shooterConf
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(SHOOTING_MOTOR_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(false);
    shooterConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    tryUntilOk(
        leftShootingMotor,
        5,
        () ->
            leftShootingMotor.configure(
                shooterConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(leftShootingMotor, 5, () -> shootingEncoder.setPosition(0));

    followerConf
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(SHOOTING_MOTOR_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .follow(LEFT_SHOOTING_MOTOR_ID, true);

    followerConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    tryUntilOk(
        rightShootingMotor,
        5,
        () ->
            rightShootingMotor.configure(
                followerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(rightShootingMotor, 5, () -> shootingEncoder.setPosition(0));

    turnConf
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TURNING_MOTOR_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(true);
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

    inputs.shootingMotorConnected = shootDebounce.calculate(!sparkStickyFault);

    ifOk(
        leftShootingMotor,
        shootingEncoder::getVelocity,
        (value) -> {
          inputs.shootingMotorSpeedRadsPerSecond = value;
          inputs.shooterReady = value >= SHOOTER_TARGET_SPEED_RADS;
        });
    ifOk(
        leftShootingMotor,
        new DoubleSupplier[] {
          leftShootingMotor::getBusVoltage, leftShootingMotor::getAppliedOutput
        },
        (value) -> inputs.shootingMotorAppliedVolts = value[0] * value[1]);
    ifOk(
        leftShootingMotor,
        leftShootingMotor::getOutputCurrent,
        (value) -> inputs.shootingMotorCurrentAmps = value);

    inputs.turningMotorConnected = turnDebounce.calculate(!sparkStickyFault);

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
    leftShootingMotor.set(speed);
  }

  public void turnHoodAngle(double angleDegrees) {
    turningController.setSetpoint(angleDegrees, ControlType.kPosition);
  }

  public void launcherWarning() {
    if (leftShootingMotor.getMotorTemperature() >= SHOOTER_TEMP_HARD_LIMIT) {
      Elastic.sendNotification(
          new Notification()
              .withLevel(Elastic.NotificationLevel.WARNING)
              .withTitle("MOTOR OVERHEAT WARNING")
              .withDescription("MOTOR IS EXTREMELY HOT. PROCESS WITH CAUTION.")
              .withDisplaySeconds(8.0));
    }
  }
}
