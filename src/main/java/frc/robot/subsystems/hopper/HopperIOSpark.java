package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class HopperIOSpark implements HopperIO {

  private final SparkBase hopperMotor;
  private final RelativeEncoder hopperEncoder;

  private final SparkBase indexerMotor;
  private final RelativeEncoder indexerEncoder;

  private final Debouncer indexerDebounce = new Debouncer(0.05);

  public HopperIOSpark() {
    hopperMotor = new SparkFlex(HOPPER_MOTOR_ID, MotorType.kBrushless);
    hopperEncoder = hopperMotor.getEncoder();

    indexerMotor = new SparkFlex(INDEXER_MOTOR_ID, MotorType.kBrushless);
    indexerEncoder = indexerMotor.getEncoder();

    SparkFlexConfig HopperConf = new SparkFlexConfig();
    HopperConf.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(HOPPER_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(true);
    HopperConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    tryUntilOk(
        hopperMotor,
        5,
        () ->
            hopperMotor.configure(
                HopperConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkFlexConfig indexerConf = new SparkFlexConfig();
    indexerConf
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(INDEXER_MOTOR_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(false);
    indexerConf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    tryUntilOk(
        indexerMotor,
        5,
        () ->
            indexerMotor.configure(
                indexerConf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(hopperMotor, 5, () -> hopperEncoder.setPosition(0));
    tryUntilOk(indexerMotor, 5, () -> indexerEncoder.setPosition(0));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    boolean sparkStickyFault = false;

    ifOk(
        hopperMotor,
        hopperEncoder::getVelocity,
        (value) -> inputs.hopperMotorSpeedRadsPerSecond = value);
    ifOk(
        hopperMotor,
        new DoubleSupplier[] {hopperMotor::getBusVoltage, hopperMotor::getAppliedOutput},
        (value) -> inputs.hopperMotorAppliedVolts = value[0] * value[1]);
    ifOk(
        hopperMotor,
        hopperMotor::getOutputCurrent,
        (value) -> inputs.hopperMotorCurrentAmps = value);
    inputs.hopperMotorConnected = indexerDebounce.calculate(!sparkStickyFault);

    ifOk(
        indexerMotor,
        indexerEncoder::getVelocity,
        (value) -> inputs.indexerMotorSpeedRadsPerSecond = value);
    ifOk(
        indexerMotor,
        new DoubleSupplier[] {indexerMotor::getBusVoltage, indexerMotor::getAppliedOutput},
        (value) -> inputs.indexerMotorAppliedVolts = value[0] * value[1]);
    ifOk(
        indexerMotor,
        indexerMotor::getOutputCurrent,
        (value) -> inputs.indexerMotorCurrentAmps = value);
    inputs.indexerMotorConneced = indexerDebounce.calculate(!sparkStickyFault);
  }

  @Override
  public void setHopperSpeed(double speed) {
    if (motorIsOverTemp()) {
      hopperMotor.set(HOPPER_OVERHEAT_SPEED);
      return;
    }
    hopperMotor.set(speed);
  }

  @Override
  public void setIndexerSpeed(double speed) {
    indexerMotor.set(speed);
  }

  @Override
  public boolean motorIsOverTemp() {
    return hopperMotor.getMotorTemperature() >= HOPPER_TEMPERATURE_HARD_STOP;
  }
}
