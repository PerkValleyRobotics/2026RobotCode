package frc.robot.util;

import java.util.Optional;
import java.util.function.DoubleSupplier;

public class AngleSupplier implements DoubleSupplier {

  public class AngleRange {
    double max, min;

    public AngleRange(double min, double max) {
      if (min > max) {
        throw new IllegalArgumentException("Invalid Angle Range!");
      }
      this.max = max;
      this.min = min;
    }

    public AngleRange() {
      this.max = 360;
      this.min = 0;
    }
  }
  ;

  private double value = 0.0;
  private AngleRange range;
  private Optional<AngleRange> mappedRange;

  public AngleSupplier(double value) {
    this.value = value;
    this.range = new AngleRange();
  }

  public AngleSupplier(double value, AngleRange range, Optional<AngleRange> mappedRange) {
    this.value = value;
    this.range = range;
    this.mappedRange = mappedRange;
  }

  @Override
  public double getAsDouble() {
    return value;
  }

  public double getAsWrappedDouble() {
    if (value > range.max) {
      return (double) (value % range.max);
    } else if (value < range.min) {
      return range.max - (double) (Math.abs(value) % range.max);
    }
    return getAsDouble();
  }

  public double getRemappedAngleAsDouble() {
    if (mappedRange.isEmpty()) {
      return getAsWrappedDouble();
    }
    return (getAsWrappedDouble() - range.min)
            * (mappedRange.get().max - mappedRange.get().min)
            / (range.max - range.min)
        + mappedRange.get().min;
  }
}
