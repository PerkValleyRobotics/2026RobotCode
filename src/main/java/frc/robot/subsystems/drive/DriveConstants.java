// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

  // put a limiter here in order to reduce the speed of the drivetrain
  public static final double MAX_SPEED_METERS_PER_SEC = 4.8;

  public static final double ODOMETRY_FREQUENCY = 100.0; // Hz
  public static final double TRACK_WIDTH = Units.inchesToMeters(22.162);
  public static final double WHEEL_BASE = Units.inchesToMeters(22.162);
  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
        new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-3.117); // 0
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(-2.984); // 1
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(-0.040); // 2
  public static final Rotation2d backRightZeroRotation = new Rotation2d(-2.201); // 3

  // Device CAN IDs
  public static final int FRONT_LEFT_DRIVE_ID = 1;
  public static final int FRONT_RIGHT_DRIVE_ID = 11;
  public static final int BACK_LEFT_DRIVE_ID = 21;
  public static final int BACK_RIGHT_DRIVE_ID = 31;

  public static final int FRONT_LEFT_TURN_ID = 2;
  public static final int FRONT_RIGHT_TURN_ID = 12;
  public static final int BACK_LEFT_TURN_ID = 22;
  public static final int BACK_RIGHT_TURN_ID = 32;

  public static final int FRONT_LEFT_CANCODER_ID = 3;
  public static final int FRONT_RIGHT_CANCODER_ID = 13;
  public static final int BACK_LEFT_CANCODER_ID = 23;
  public static final int BACK_RIGHT_CANCODER_ID = 33;
  // Drive motor configuration
  public static final int DRIVE_MOTOR_MAX_AMPERAGE = 50;
  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
  public static final double DRIVE_MOTOR_REDUCTION = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  // ported from last year
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double DRIVE_ENCODER_POSITION_FACTOR =
      2 * Math.PI / DRIVE_MOTOR_REDUCTION; // Rotor Rotations ->
  // Wheel Radians
  public static final double DRIVE_ENCODER_VELOCITY_FACTOR =
      (2 * Math.PI) / 60.0 / DRIVE_MOTOR_REDUCTION; // Rotor RPM ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double DRIVE_KP = 0.0;
  public static final double DRIVE_KD = 0.0;
  public static final double DRIVE_KS = 0.0;
  public static final double DRIVE_KV = 0.1;
  public static final double DRIVE_SIMP = 0.05;
  public static final double DRIVE_SIMD = 0.0;
  public static final double DRIVE_SIMKS = 0.0;
  public static final double DRIVE_SIMKV = 0.0789;

  // Turn motor configuration
  public static final boolean TURN_INVERTED = true;
  public static final int TURN_MOTOR_MAX_AMPERAGE = 50;
  public static final double TURN_MOTOR_REDUCTION = 150 / 7;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);
  // Turn encoder configuration
  public static final boolean TURN_ENCODER_INVERTED = true;
  public static final double TURN_ENCODER_POSITION_FACTOR =
      2 * Math.PI / TURN_MOTOR_REDUCTION; // Rotations -> Radians
  public static final double TURN_ENCODER_VELOCITY_FACTOR =
      (2 * Math.PI) / 60.0 / TURN_MOTOR_REDUCTION; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double TURN_KP = 0.5;
  public static final double TURN_KD = 0.1;
  public static final double TURN_SIMP = 8.0;
  public static final double TURN_SIMD = 0.0;
  public static final double TURN_PID_MIN_INPUT = 0; // Radians
  public static final double TURN_PID_MAX_INPUT = 2 * Math.PI; // Radians

  // PathPlanner configuration
  public static final double ROBOT_MASS_KG = 74.088;
  public static final double ROBOT_MOI = 6.883;
  public static final double WHEEL_COF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              WHEEL_RADIUS_METERS,
              MAX_SPEED_METERS_PER_SEC,
              WHEEL_COF,
              driveGearbox.withReduction(DRIVE_MOTOR_REDUCTION),
              DRIVE_MOTOR_MAX_AMPERAGE,
              1),
          moduleTranslations);
}
