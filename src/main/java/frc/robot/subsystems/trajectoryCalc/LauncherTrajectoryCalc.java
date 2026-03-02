package frc.robot.subsystems.trajectoryCalc;

// leaves space open for multiple trajectory implementations, along with multiparameter inputs
public interface LauncherTrajectoryCalc {
  public record inputParameters(double[] location, double velocityVectors) {}
  ;

  public default double calculateAngleTrajectory(double[] location) {
    return 0;
  }
  ;
}
