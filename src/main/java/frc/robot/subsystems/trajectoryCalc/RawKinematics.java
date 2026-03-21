package frc.robot.subsystems.trajectoryCalc;

import com.fasterxml.jackson.core.exc.InputCoercionException;

import frc.robot.subsystems.drive.Drive;

// raw calculations done using highschool physics
// note to self make freshman do algebra labor for phun

public class RawKinematics implements LauncherTrajectoryCalc {

  public static double calculate(inputParameters initialParameters) {
    double height;

    return 0;
  }

  public static inputParameters getInputparameters(Drive drive) {
    return new inputParameters(drive.getPose(), drive.getRotation(), drive.getFFCharacterizationVelocity());
  }
}
