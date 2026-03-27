package frc.robot.subsystems.launcher;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class LauncherConstants {
  public static final int TOP_LEFT_SHOOTING_MOTOR_ID = 40;
  public static final int BOTTOM_LEFT_SHOOTING_MOTOR_ID = 44;

  public static final int TOP_RIGHT_SHOOTING_MOTOR_ID = 42;
  public static final int BOTTOM_RIGHT_SHOOTING_MOTOR_ID = 43;

  public static final int TURNING_MOTOR_ID = 41;

  public static final int SHOOTING_MOTOR_MAX_AMPERAGE = 40;
  public static final int TURNING_MOTOR_MAX_AMPERAGE = 40;

  public static final double SHOOTING_MOTOR_SPEED = .725; // 0.8, 80;
  public static final double SHOOTER_TARGET_SPEED_RADS = 3100;
  public static final double SHOOTER_TEMP_HARD_LIMIT = 100;

  // flywheel contants
  public static final double SHOOT_MOTOR_kP = 0.5;
  public static final double SHOOT_MOTOR_kI = 0;
  public static final double SHOOT_MOTOR_kD = 0;
  public static final double SHOOT_MOTOR_kFF = 0;

  // turn PID
  public static final double TURN_MOTOR_kP = 0.5;
  public static final double TURN_MOTOR_kI = 0;
  public static final double TURN_MOTOR_kD = 0;
  public static final double TURN_MOTOR_kFF = 0;

  // simulation items
  public static DCMotor shootingSimMotor = DCMotor.getNEO(1);
  public static DCMotor turningSimMotor = DCMotor.getNEO(1);

  // StateSpace Variables
  public static final double STATE_SPACE_CYCLE_DT = 0.02;

  // Launcher StateSpace Variables
  public static final int FLYWHEEL_GEARING = 1;
  public static final double FLYWHEEL_MOI_KGM = 0.02107;
  public static final double FLYWHEEL_QELMS = 8.00; // Velocity Error Tolerance (Rad/S)
  public static final double FLYWHEEL_RELMS = 12.00; // Voltage Control Effort (Volts)
  public static final double FLYWHEEL_CONFIDENCE_VALUE = 3; // confidence in our statespace model
  public static final double FLYWHEEL_ENCODER_CONFIDENCE =
      0.03; // confidence in our input (encoder)

  // Hood StateSpace Variables
  public static final int HOOD_GEARING = 1;
  public static final double HOOD_MOI_KGM = 1;
  public static final double HOOD_QELMS = 8.0;
  // Position and Velocity Error Tolerance (May need two parameters)
  public static final double HOOD_RELMS = 12.00; // Voltage Control Effort (Volts)
  public static final double HOOD_CONFIDENCE_VALUE = 3; // confidence in our statespace model
  public static final double HOOD_ENCODER_CONFIDENCE = 0.03; // confidence in our input (encoder)

  public static final double TURN_MOTOR_SIM_kP = 0;
  public static final double TURN_MOTOR_SIM_kI = 0;
  public static final double TURN_MOTOR_SIM_kD = 0;
  public static final double TURN_MOTOR_SIM_kFF = 0;

  public static final double LAUNCHER_HEIGHT = Units.inchesToMeters(18);
  public static final double HOOD_ANGLE_MAX_LIMIT = 90;
  public static final double HOOD_ANGLE_MIN_LIMIT = 0;
  public static final double HOOD_MAX_SETPOINT = 1;
  public static final double HOOD_MIN_SETPOINT = 0;
}
