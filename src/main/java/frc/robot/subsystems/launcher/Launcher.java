// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static frc.robot.subsystems.launcher.LauncherConstants.FLYWHEEL_GEARING;
import static frc.robot.subsystems.launcher.LauncherConstants.FLYWHEEL_MOI_KGM;
import static frc.robot.subsystems.launcher.LauncherConstants.FLYWHEEL_QELMS;
import static frc.robot.subsystems.launcher.LauncherConstants.FLYWHEEL_RELMS;
import static frc.robot.subsystems.launcher.LauncherConstants.HOOD_ANGLE_MAX_LIMIT;
import static frc.robot.subsystems.launcher.LauncherConstants.HOOD_ANGLE_MIN_LIMIT;
import static frc.robot.subsystems.launcher.LauncherConstants.HOOD_GEARING;
import static frc.robot.subsystems.launcher.LauncherConstants.HOOD_MAX_SETPOINT;
import static frc.robot.subsystems.launcher.LauncherConstants.HOOD_MIN_SETPOINT;
import static frc.robot.subsystems.launcher.LauncherConstants.HOOD_MOI_KGM;
import static frc.robot.subsystems.launcher.LauncherConstants.HOOD_QELMS;
import static frc.robot.subsystems.launcher.LauncherConstants.STATE_SPACE_CYCLE_DT;
import static frc.robot.subsystems.launcher.LauncherConstants.turningSimMotor;

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
import frc.robot.subsystems.trajectoryCalc.LauncherTrajectoryCalc.inputParameters;
import frc.robot.util.AngleSupplier;
import frc.robot.util.AngleSupplier.AngleRange;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

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

  public Launcher(LauncherIO io, LauncherTrajectoryCalc trajectoryCalculator) {

    this.io = io;
    this.flywheel =
        LinearSystemId.createFlywheelSystem(turningSimMotor, FLYWHEEL_MOI_KGM, FLYWHEEL_GEARING);
    this.hood =
        LinearSystemId.createSingleJointedArmSystem(turningSimMotor, HOOD_MOI_KGM, HOOD_GEARING);

    // sysid configuration
    // this.sysId = new SysIdRoutine(, null))

    // filter and kqr
    this.flywheelObservFilter =
        new KalmanFilter<>(
            Nat.N1(), Nat.N1(), flywheel, VecBuilder.fill(3), VecBuilder.fill(0.03), 0.02);
    this.hoodObservFilter =
        new KalmanFilter<>(
            Nat.N2(), Nat.N2(), hood, VecBuilder.fill(3, 3), VecBuilder.fill(0.03, 0.03), 0.02);

    this.flywheelController =
        new LinearQuadraticRegulator<>(
            flywheel,
            VecBuilder.fill(
                FLYWHEEL_QELMS), // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave
            // more
            // aggressively.
            VecBuilder.fill(
                FLYWHEEL_RELMS), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12
            // is a good
            // starting point because that is the (approximate) maximum voltage of a
            // battery.
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
    // lower if using notifiers.

    this.hoodController =
        new LinearQuadraticRegulator<>(
            hood,
            VecBuilder.fill(HOOD_QELMS, HOOD_QELMS),
            VecBuilder.fill(12.0),
            STATE_SPACE_CYCLE_DT);

    this.flywheelControlLoop =
        new LinearSystemLoop<N1, N1, N1>(
            flywheel, flywheelController, flywheelObservFilter, 12.0, 0.020);
    this.hoodControlLoop =
        new LinearSystemLoop<N2, N1, N2>(hood, hoodController, hoodObservFilter, 12.0, 0.020);

    setLauncherTrajectoryCalculator(trajectoryCalculator);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);

    // state space update
    this.flywheelControlLoop.correct(VecBuilder.fill(io.getVelocityRadsPerSec()));

    this.flywheelControlLoop.predict(STATE_SPACE_CYCLE_DT);
    double nextVoltage = flywheelControlLoop.getU(0);
    // io.setVoltage(nextVoltage);
  }

  // actual functions for launcher
  public void runLauncher(double speed) {
    io.setShooterSpeed(speed);
  }

  public void stopLauncher() {
    io.setShooterSpeed(0);
  }

  public void turnHoodSetpoint(double setpoint) {
    this.setpoint = setpoint;
    io.setHoodAngleSetpoint(setpoint);
  }

  public void incrementHoodSetpoint() {
    io.setHoodAngleSetpoint(++this.setpoint);
  }

  public void resetHoodSetpoint(double setpoint) {
    this.setpoint = HOOD_ANGLE_MIN_LIMIT;
    io.setHoodAngleSetpoint(setpoint);
  }

  // trajectory
  public void setLauncherTrajectoryCalculator(LauncherTrajectoryCalc trajectoryCalculator) {
    this.trajectoryCalculator = trajectoryCalculator;
  }

  public void calculateAndSetTrajectoryAngle(inputParameters inputs) {
    if (trajectoryCalculator.getClass() != null) {
      AngleSupplier angle =
          new AngleSupplier(
              trajectoryCalculator.calculate(inputs),
              new AngleRange(HOOD_ANGLE_MIN_LIMIT, HOOD_ANGLE_MAX_LIMIT),
              Optional.of(new AngleRange(HOOD_MIN_SETPOINT, HOOD_MAX_SETPOINT)));
      turnHoodSetpoint(angle.getRemappedAngleAsDouble());
    }
  }

  // STATE SPACE METHODS
  public void setLauncherStateReferencePoint(double speed) {
    this.flywheelControlLoop.setNextR(VecBuilder.fill(speed));
  }
}
