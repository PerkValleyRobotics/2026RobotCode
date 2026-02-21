// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.launcher;

import static frc.robot.subsystems.launcher.LauncherConstants.HOOD_ANGLE_MIN_LIMIT;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  LauncherIO io;
  LauncherIOInputsAutoLogged inputs;

  double setpoint;

  // constructor
  public Launcher(LauncherIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);
  }

  // actual functions for launcher
  public void runLauncher(double speed) {
    io.setShooterSpeed(speed);
  }

  public void stopLauncher(double speed) {
    io.setShooterSpeed(0);
  }

  public void turnHoodAngle(double angleDegrees) {
    this.setpoint = angleDegrees;
    io.turnHoodAngle(angleDegrees);
  }

  public void resetHoodAngle(double angleDegrees) {
    this.setpoint = HOOD_ANGLE_MIN_LIMIT;
    io.turnHoodAngle(HOOD_ANGLE_MIN_LIMIT);
  }
}