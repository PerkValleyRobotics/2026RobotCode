// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static frc.robot.subsystems.launcher.LauncherConstants.HOOD_ANGLE_MIN_LIMIT;
import static frc.robot.subsystems.launcher.LauncherConstants.shootingSimMotor;
import static frc.robot.subsystems.launcher.LauncherConstants.turningSimMotor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.trajectoryCalc.LauncherTrajectoryCalc;

public class Launcher extends SubsystemBase {
  private LauncherIO io;
  private LauncherTrajectoryCalc trajectoryCalculator;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  // first linear system implementation (could be wrong)
  /*
   * a linear system is defined by three parameters:
   * number of states: Not the states of a system, but rather the states your
   * tracking
   * number of inputs: this can be stuff like voltage.
   * number of outputs: outputs we actually care about.
   */
  private final LinearSystem<N1, N1, N1> flywheel;
  private final LinearSystem<N2, N1, N2> hood;

  /* kalman filters and lqrs for filtering and managing control rules */
  private final KalmanFilter<N1, N1, N1> flywheelObservFilter;
  private final LinearQuadraticRegulator<N1, N1, N1> flywheelController;
  private final LinearSystemLoop<N1, N1, N1> flywheelControlLoop;

  private final KalmanFilter<N2, N1, N2> hoodObservFilter;
  private final LinearQuadraticRegulator<N2, N1, N2> hoodController;
  private final LinearSystemLoop<N2, N1, N2> hoodControlLoop;

  // use depending on conditions of the calculated parameters
  private SysIdRoutine sysIdFlywheel;
  private SysIdRoutine sysIdHood;

  double setpoint;

  // constructor
  public Launcher(LauncherIO io) {
    this.io = io;
    this.flywheel = LinearSystemId.createFlywheelSystem(shootingSimMotor, 0, 0);
    this.hood = LinearSystemId.createSingleJointedArmSystem(turningSimMotor, 0, 0);

    // sysid configuration
    // this.sysId = new SysIdRoutine(, null))

    // filter and kqr
    this.flywheelObservFilter = new KalmanFilter<>(Nat.N1(), Nat.N1(), flywheel, VecBuilder.fill(3),
        VecBuilder.fill(0.03),
        0.02);
    this.hoodObservFilter = new KalmanFilter<>(Nat.N2(), Nat.N2(), hood, VecBuilder.fill(3, 3),
        VecBuilder.fill(0.03, 0.03), 0.02);

    this.flywheelController = new LinearQuadraticRegulator<>(
        flywheel,
        VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
        // this to more heavily penalize state excursion, or make the controller behave
        // more
        // aggressively.
        VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
        // heavily penalize control effort, or make the controller less aggressive. 12
        // is a good
        // starting point because that is the (approximate) maximum voltage of a
        // battery.
        0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
    // lower if using notifiers.

    this.hoodController = new LinearQuadraticRegulator<>(hood, VecBuilder.fill(8, 8), VecBuilder.fill(12.0), 0.020);

    this.flywheelControlLoop = new LinearSystemLoop<N1, N1, N1>(flywheel, flywheelController, flywheelObservFilter,
        12.0, 0.020);
    this.hoodControlLoop = new LinearSystemLoop<N2, N1, N2>(hood, hoodController, hoodObservFilter, 12.0, 0.020);

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.launcherWarning();
    Logger.processInputs("Launcher", inputs);
  }

  // actual functions for launcher
  public void runLauncher(double speed) {
    io.setShooterSpeed(speed);
  }

  public void stopLauncher() {
    io.setShooterSpeed(0);
  }

  public void turnHoodAngle(double angleDegrees) {
    this.setpoint = angleDegrees;
    io.turnHoodAngle(setpoint);
  }

  public void incrementHoodAngle() {
    io.turnHoodAngle(++this.setpoint);
  }

  public void resetHoodAngle(double angleDegrees) {
    this.setpoint = HOOD_ANGLE_MIN_LIMIT;
    io.turnHoodAngle(setpoint);
  }
}
