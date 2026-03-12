package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeIOConstants.*;
import static frc.robot.subsystems.launcher.LauncherConstants.TURNING_MOTOR_MAX_AMPERAGE;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import java.util.function.DoubleSupplier;

public class IntakeIOSpark implements IntakeIO {
  private final SparkBase leftTurnMotor;
  private final SparkBase rightTurnMotor;
  private final SparkBase intakeRunMotor;
  private final RelativeEncoder leftTurnEncoder;
  private final RelativeEncoder rightTurnEncoder;
  private final RelativeEncoder runEncoder;

  private final Debouncer runDebounce = new Debouncer(0.05);
  private final Debouncer turnDebounce = new Debouncer(0.05);

  private SparkClosedLoopController turningController;

  public IntakeIOSpark() {
    leftTurnMotor = new SparkMax(INTAKE_LEFT_TURN_MOTOR_ID, MotorType.kBrushless);
    rightTurnMotor = new SparkMax(INTAKE_RIGHT_TURN_MOTOR_ID, MotorType.kBrushless);
    intakeRunMotor = new SparkMax(INTAKE_RUN_MOTOR_ID, MotorType.kBrushless);

    leftTurnEncoder = leftTurnMotor.getEncoder();
    rightTurnEncoder = rightTurnMotor.getEncoder();
    runEncoder = intakeRunMotor.getEncoder();

    turningController = leftTurnMotor.getClosedLoopController();

    SparkMaxConfig turnConf = new SparkMaxConfig();

    turnConf
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(TURNING_MOTOR_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(false);
    turnConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    tryUntilOk(
        leftTurnMotor,
        5,
        () ->
            leftTurnMotor.configure(
                turnConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(leftTurnMotor, 5, () -> leftTurnEncoder.setPosition(0));

    turnConf
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(INTAKE_TURN_MOTOR_kP)
        .i(INTAKE_TURN_MOTOR_kI)
        .d(INTAKE_TURN_MOTOR_kD);

    // optionally switch direction here
    SparkMaxConfig followConf = new SparkMaxConfig();

    followConf
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(TURNING_MOTOR_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(true)
        .follow(INTAKE_LEFT_TURN_MOTOR_ID);
    followConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    //
    tryUntilOk(
        rightTurnMotor,
        5,
        () ->
            rightTurnMotor.configure(
                turnConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(rightTurnMotor, 5, () -> rightTurnEncoder.setPosition(0));

    SparkMaxConfig runConf = new SparkMaxConfig();

    runConf
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(INTAKE_RUN_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(true);
    runConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    tryUntilOk(
        intakeRunMotor,
        5,
        () ->
            intakeRunMotor.configure(
                runConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    tryUntilOk(intakeRunMotor, 5, () -> runEncoder.setPosition(0));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    boolean sparkStickyFault = false;

    inputs.intakeMotorConnected = runDebounce.calculate(!sparkStickyFault);

    ifOk(
        intakeRunMotor,
        runEncoder::getVelocity,
        (value) -> inputs.intakeMotorSpeedRadsPerSecond = value);

    ifOk(
        intakeRunMotor,
        new DoubleSupplier[] {intakeRunMotor::getBusVoltage, intakeRunMotor::getAppliedOutput},
        (value) -> inputs.intakeMotorAppliedVolts = value[0] * value[1]);
    ifOk(
        intakeRunMotor,
        intakeRunMotor::getOutputCurrent,
        (value) -> inputs.intakeMotorCurrentAmps = value);

    inputs.turnLeftIntakeMotorConnected = turnDebounce.calculate(!sparkStickyFault);
    inputs.turnRightIntakeMotorConnected = turnDebounce.calculate(!sparkStickyFault);

    ifOk(
        leftTurnMotor,
        leftTurnEncoder::getPosition,
        (value) -> inputs.turnIntakeMotorPositionDeg = value);

    ifOk(
        leftTurnMotor,
        new DoubleSupplier[] {leftTurnMotor::getBusVoltage, leftTurnMotor::getAppliedOutput},
        (value) -> inputs.turnLeftIntakeMotorAppliedVolts = value[0] * value[1]);
    ifOk(
        leftTurnMotor,
        leftTurnMotor::getOutputCurrent,
        (value) -> inputs.turnLeftIntakeMotorCurrentAmps = value);

    ifOk(
        rightTurnMotor,
        new DoubleSupplier[] {rightTurnMotor::getBusVoltage, rightTurnMotor::getAppliedOutput},
        (value) -> inputs.turnRightIntakeMotorAppliedVolts = value[0] * value[1]);
    ifOk(
        rightTurnMotor,
        rightTurnMotor::getOutputCurrent,
        (value) -> inputs.turnRightIntakeMotorCurrentAmps = value);
  }

  public void setIntakeSpeed(double speed) {
    intakeRunMotor.set(speed);
  }

  public void turnIntakeMotorAngle(double degrees) {
    turningController.setSetpoint(degrees, ControlType.kPosition);
  }
}
