package frc.robot.subsystems.trajectoryCalc;

import static frc.robot.subsystems.trajectoryCalc.TrajectoriesConstants.G;

public class FreshmenImplmentation implements LauncherTrajectoryCalc {

  public double calculateAngleTrajectory(inputParameters initialParameters) {
    // theta = 1/2 * arcsin((9.81 * distance_to_hub)/launchVelocity^2
    return (1 / 2) * Math.asin((G / Math.pow(initialParameters.launchVelocity(), 2)));
  }
  ;
}
