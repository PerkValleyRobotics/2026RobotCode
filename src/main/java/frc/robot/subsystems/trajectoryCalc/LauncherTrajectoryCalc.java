package frc.robot.subsystems.trajectoryCalc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.DoubleSupplier;

// leaves space open for multiple trajectory implementations, along with multiparameter inputs

public interface LauncherTrajectoryCalc {
  public record inputParameters(double launchVelocity, Pose2d currentPose, Translation2d velocityVectors) {

  };
}
