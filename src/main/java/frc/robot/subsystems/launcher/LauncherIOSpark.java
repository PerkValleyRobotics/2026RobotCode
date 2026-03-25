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
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import java.util.function.DoubleSupplier;

public class LauncherIOSpark implements LauncherIO {
  private final SparkBase topLeftShootingMotor;
  private final SparkBase topRightShootingMotor;
  private final SparkBase bottomLeftShootingMotor;
  private final SparkBase bottomRightShootingMotor;

  private final SparkBase turningMotor;
  private final RelativeEncoder shootingEncoder;
  private final RelativeEncoder turningEncoder;

  private final Debouncer shootDebounce = new Debouncer(0.05);
  private final Debouncer turnDebounce = new Debouncer(0.05);

  private SparkClosedLoopController turningController;

  public LauncherIOSpark() {
    topLeftShootingMotor = new SparkMax(TOP_LEFT_SHOOTING_MOTOR_ID, MotorType.kBrushless);
    topRightShootingMotor = new SparkMax(TOP_RIGHT_SHOOTING_MOTOR_ID, MotorType.kBrushless);
    bottomLeftShootingMotor = new SparkMax(BOTTOM_LEFT_SHOOTING_MOTOR_ID, MotorType.kBrushless);
    bottomRightShootingMotor = new SparkMax(BOTTOM_RIGHT_SHOOTING_MOTOR_ID, MotorType.kBrushless);

    turningMotor = new SparkFlex(TURNING_MOTOR_ID, MotorType.kBrushless);

    shootingEncoder = topLeftShootingMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    turningController = turningMotor.getClosedLoopController();

    SparkBaseConfig mainShooterConf = new SparkMaxConfig();
    SparkBaseConfig followerConf = new SparkMaxConfig();
    SparkBaseConfig turnConf = new SparkMaxConfig();

    mainShooterConf
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(SHOOTING_MOTOR_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(false);
    mainShooterConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    tryUntilOk(
        topLeftShootingMotor,
        5,
        () -> topLeftShootingMotor.configure(
            mainShooterConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(topLeftShootingMotor, 5, () -> shootingEncoder.setPosition(0));

    mainShooterConf.inverted(true);

    tryUntilOk(
        topRightShootingMotor,
        5,
        () -> topRightShootingMotor.configure(
            mainShooterConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(topRightShootingMotor, 5, () -> shootingEncoder.setPosition(0));

    followerConf
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(SHOOTING_MOTOR_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .follow(TOP_LEFT_SHOOTING_MOTOR_ID, true);

    followerConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    followerConf.inverted(true);

    tryUntilOk(
        bottomLeftShootingMotor,
        5,
        () -> bottomLeftShootingMotor.configure(
            followerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(bottomLeftShootingMotor, 5, () -> shootingEncoder.setPosition(0));

    followerConf.follow(TOP_RIGHT_SHOOTING_MOTOR_ID, true);
    followerConf.inverted(true);

    tryUntilOk(
        bottomRightShootingMotor,
        5,
        () -> bottomRightShootingMotor.configure(
            followerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(bottomRightShootingMotor, 5, () -> shootingEncoder.setPosition(0));

    turnConf
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(TURNING_MOTOR_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(true);
    turnConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    turnConf.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(TURN_MOTOR_kP)
        .i(TURN_MOTOR_kI)
        .d(TURN_MOTOR_kD);

    tryUntilOk(
        turningMotor,
        5,
        () -> turningMotor.configure(
            turnConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(turningMotor, 5, () -> turningEncoder.setPosition(0));
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    boolean sparkStickyFault = false;

    inputs.shootingMotorConnected = shootDebounce.calculate(!sparkStickyFault);

    ifOk(
        topLeftShootingMotor,
        shootingEncoder::getVelocity,
        (value) -> {
          inputs.shootingMotorSpeedRadsPerSecond = value;
          inputs.shooterReady = value >= SHOOTER_TARGET_SPEED_RADS;
        });
    ifOk(
        topLeftShootingMotor,
        new DoubleSupplier[] {
            topLeftShootingMotor::getBusVoltage, topLeftShootingMotor::getAppliedOutput
        },
        (value) -> inputs.shootingMotorAppliedVolts = value[0] * value[1]);
    ifOk(
        topLeftShootingMotor,
        topLeftShootingMotor::getOutputCurrent,
        (value) -> inputs.shootingMotorCurrentAmps = value);

    inputs.turningMotorConnected = turnDebounce.calculate(!sparkStickyFault);

    ifOk(
        turningMotor,
        turningEncoder::getPosition,
        (value) -> inputs.turningMotorPositionDeg = value);
    ifOk(
        turningMotor,
        new DoubleSupplier[] { turningMotor::getBusVoltage, turningMotor::getAppliedOutput },
        (value) -> inputs.turningMotorAppliedVolts = value[0] * value[1]);
    ifOk(
        turningMotor,
        turningMotor::getOutputCurrent,
        (value) -> inputs.turningMotorCurrentAmps = value);
  }

  @Override
  public void setShooterSpeed(double speed) {
    topLeftShootingMotor.set(-speed);
    topRightShootingMotor.set(-speed);
  }

  public void setHoodAngleSetpoint(double setpoint) {
    turningController.setSetpoint(setpoint, ControlType.kPosition);
  }

  public void launcherWarning() {
    if (topLeftShootingMotor.getMotorTemperature() >= SHOOTER_TEMP_HARD_LIMIT) {
      Elastic.sendNotification(
          new Notification()
              .withLevel(Elastic.NotificationLevel.WARNING)
              .withTitle("MOTOR OVERHEAT WARNING")
              .withDescription("MOTOR IS EXTREMELY HOT. PROCESS WITH CAUTION.")
              .withDisplaySeconds(8.0));
    }
  }

  public void setVoltage(double voltage) {
    topLeftShootingMotor.setVoltage(voltage);
  }

  public double getVelocityRadsPerSec() {
    return Units.rotationsPerMinuteToRadiansPerSecond(shootingEncoder.getVelocity());
  }
}
