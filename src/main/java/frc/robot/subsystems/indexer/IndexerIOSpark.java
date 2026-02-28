package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import java.util.function.DoubleSupplier;

public class IndexerIOSpark implements IndexerIO {
  private final SparkBase indexerMotor;
  private final RelativeEncoder indexerEncoder;

  private final Debouncer indexerDebounce = new Debouncer(0.05);

  public IndexerIOSpark() {
    indexerMotor = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);
    indexerEncoder = indexerMotor.getEncoder();

    SparkMaxConfig conf = new SparkMaxConfig();
    conf.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(INDEXER_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(false);
    conf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    tryUntilOk(
        indexerMotor,
        5,
        () ->
            indexerMotor.configure(
                conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(indexerMotor, 5, () -> indexerEncoder.setPosition(0));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    boolean sparkStickyFault = false;

    inputs.indexerMotorConnected = indexerDebounce.calculate(sparkStickyFault);
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
  }

  @Override
  public void setIndexerSpeed(double speed) {
    indexerMotor.set(speed);
  }
}
