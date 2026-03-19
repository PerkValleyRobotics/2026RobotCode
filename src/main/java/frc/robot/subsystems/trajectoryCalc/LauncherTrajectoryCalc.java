package frc.robot.subsystems.trajectoryCalc;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;

// leaves space open for multiple trajectory implementations, along with multiparameter inputs

public interface LauncherTrajectoryCalc {
  public record inputParameters(Pose2d currentPose, DoubleSupplier velocityVectors) {}
  ;

  public default double calculateAngleTrajectory(inputParameters initialParameters) {
    return 0;
  }
  ;
}
