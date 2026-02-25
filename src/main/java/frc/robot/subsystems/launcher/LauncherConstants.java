package frc.robot.subsystems.launcher;

import edu.wpi.first.math.system.plant.DCMotor;

public class LauncherConstants {
  public static final int SHOOTING_MOTOR_ID = 40;
  public static final int TURNING_MOTOR_ID = 41;

  public static final int SHOOTING_MOTOR_MAX_AMPERAGE = 40;
  public static final int TURNING_MOTOR_MAX_AMPERAGE = 40;

  // simulation items
  public static DCMotor shootingSimMotor = DCMotor.getNEO(1);
  public static DCMotor turningSimMotor = DCMotor.getNEO(1);

  public static final double TURN_MOTOR_SIM_kP = 0;
  public static final double TURN_MOTOR_SIM_kI = 0;
  public static final double TURN_MOTOR_SIM_kD = 0;
  public static final double TURN_MOTOR_SIM_kFF = 0;

  public static final double HOOD_ANGLE_MAX_LIMIT = 90;
  public static final double HOOD_ANGLE_MIN_LIMIT = 0;

  // turn PID
  public static final double TURN_MOTOR_kP = 0;
  public static final double TURN_MOTOR_kI = 0;
  public static final double TURN_MOTOR_kD = 0;
  public static final double TURN_MOTOR_kFF = 0;
}
