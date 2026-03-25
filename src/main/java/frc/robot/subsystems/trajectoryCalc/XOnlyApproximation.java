package frc.robot.subsystems.trajectoryCalc;

import static frc.robot.FieldConstants.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.*;

// raw calculations done using highschool physics
// assumes endstates are the same so it technically works, but it only focuses on the x-angle, also
// ignores air resistance. Technically
// still fastest solution
// note to self make freshman do algebra labor for phun

public class XOnlyApproximation implements LauncherTrajectoryCalc {

  public double calculate(inputParameters inputs) {
    Translation2d Target =
        DriverStation.getAlliance().get() == DriverStation.Alliance.Red
            ? new Translation2d(RED_HUB_TRIANGULATED_X, HUB_TRIANGULATED_Y)
            : new Translation2d(BLUE_HUB_TRIANGULATED_X, HUB_TRIANGULATED_Y);
    Target.plus(inputs.velocityVectors());
    double targetAngle = 0;
    try {
      targetAngle =
          0.5
              * Math.asin(
                  Target.getDistance(inputs.currentPose().getTranslation()) * 9.81)
              / Math.pow(inputs.launchVelocity(), 2);
    } finally {
      targetAngle = 0;
    }

    return targetAngle;
  }
}
