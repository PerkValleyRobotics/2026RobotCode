package frc.robot.subsystems.trajectoryCalc;

import static frc.robot.FieldConstants.BLUE_HUB_TRIANGULATED_X;
import static frc.robot.FieldConstants.HUB_TRIANGULATED_Y;
import static frc.robot.FieldConstants.HUB_TRIANGULATED_Z;
import static frc.robot.FieldConstants.RED_HUB_TRIANGULATED_X;
import static frc.robot.subsystems.trajectoryCalc.TrajectoriesConstants.G;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

// quadratic solution finder

public class TrueKinematics implements LauncherTrajectoryCalc {

  // equation is tan theta = v^2 (+-) sqrt(v^4 - g(gx^2+2yv^2)/gx;
  public double calculate(inputParameters inputs) {
    Translation2d Target =
        DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            ? new Translation2d(RED_HUB_TRIANGULATED_X, HUB_TRIANGULATED_Y)
            : new Translation2d(BLUE_HUB_TRIANGULATED_X, HUB_TRIANGULATED_Y);
    Target.plus(inputs.velocityVectors());
    double distance = Target.getDistance(inputs.currentPose().getTranslation());
    // check discriminant for solution count
    double D =
        Math.pow(inputs.launchVelocity(), 4)
            - G
                * ((G * Math.pow(distance, 2))
                    + (2 * HUB_TRIANGULATED_Z * Math.pow(inputs.launchVelocity(), 2)));
    if (D < 0) {
      return 0;
    }
    return Math.atan(Math.pow(inputs.launchVelocity(), 2) + (Math.sqrt(D) / (distance * G)));
  }
}
