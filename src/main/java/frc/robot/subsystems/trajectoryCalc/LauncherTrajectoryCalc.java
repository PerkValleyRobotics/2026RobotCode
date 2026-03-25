package frc.robot.subsystems.trajectoryCalc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

// leaves space open for multiple trajectory implementations, along with multiparameter inputs

public interface LauncherTrajectoryCalc {
  public record inputParameters(
      double launchVelocity, Pose2d currentPose, Translation2d velocityVectors) {}
  ;

  public default double calculate(inputParameters inputs) {
    return 0;
  }
}
