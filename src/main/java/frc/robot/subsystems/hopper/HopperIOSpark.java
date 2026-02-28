package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.*;
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

public class HopperIOSpark implements HopperIO {

  private final SparkBase hopperMotor;
  private final RelativeEncoder hopperEncoder;

  private final Debouncer indexerDebounce = new Debouncer(0.05);

  public HopperIOSpark() {
    hopperMotor = new SparkMax(HOPPER_MOTOR_ID, MotorType.kBrushless);
    hopperEncoder = hopperMotor.getEncoder();

    SparkMaxConfig conf = new SparkMaxConfig();
    conf.idleMode(IdleMode.kCoast)
        .smartCurrentLimit(HOPPER_MAX_AMPERAGE)
        .voltageCompensation(12.0)
        .inverted(false);
    conf.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    tryUntilOk(
        hopperMotor,
        5,
        () ->
            hopperMotor.configure(
                conf, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    tryUntilOk(hopperMotor, 5, () -> hopperEncoder.setPosition(0));
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    boolean sparkStickyFault = false;

    inputs.hopperMotorConnected = indexerDebounce.calculate(sparkStickyFault);
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
  }

  @Override
  public void setHopperSpeed(double speed) {
    hopperMotor.set(speed);
  }
}
