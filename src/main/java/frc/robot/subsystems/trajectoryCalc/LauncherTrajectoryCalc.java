package frc.robot.subsystems.trajectoryCalc;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.DoubleSupplier;

// leaves space open for multiple trajectory implementations, along with multiparameter inputs

public interface LauncherTrajectoryCalc {
  public record inputParameters(Pose2d currentPose, DoubleSupplier velocityVectors) {}
  ;

  public default double calculateAngleTrajectory(inputParameters initialParameters) {
    return 0;
  }
  ;
}
