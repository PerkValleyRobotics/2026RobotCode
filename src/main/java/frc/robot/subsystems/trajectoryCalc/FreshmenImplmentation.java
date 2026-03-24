package frc.robot.subsystems.trajectoryCalc;

public class FreshmenImplmentation implements LauncherTrajectoryCalc {

    public double calculateAngleTrajectory(inputParameters initialParameters) {
        // theta = 1/2 * arcsin((9.81 * distance_to_hub)/launchVelocity^2
        return (1 / 2) * Math.asin((9.81) / Math.pow(initialParameters.launchVelocity(), 2));
    };
}
