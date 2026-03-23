package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class FieldConstants {
  public static final AprilTagFields FIELD_VERSION = AprilTagFields.k2026RebuiltAndymark;

  public static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(FIELD_VERSION);

  public static final int RED_TRENCH_BOTTOM_LEFT = 1;
  public static final int RED_TRENCH_BOTTOM_RIGHT = 12;
  public static final int RED_TRENCH_TOP_LEFT = 6;
  public static final int RED_TRENCH_TOP_RIGHT = 7;

  public static final int BLUE_TRENCH_BOTTOM_LEFT = 22;
  public static final int BLUE_TRENCH_BOTTOM_RIGHT = 23;
  public static final int BLUE_TRENCH_TOP_LEFT = 17;
  public static final int BLUE_TRENCH_TOP_RIGHT = 28;

  public static final int RED_HUB_TOP_CENTER = 8;
  public static final int RED_HUB_TOP_OFFSET = 5;
  public static final int RED_HUB_BOTTOM_CENTER = 11;
  public static final int RED_HUB_BOTTOM_OFFSET = 2;
  public static final int RED_HUB_LEFT_TOP = 9;
  public static final int RED_HUB_LEFT_BOTTOM = 10;
  public static final int RED_HUB_RIGHT_TOP = 4;
  public static final int RED_HUB_RIGHT_BOTTOM = 3;
  public static final double RED_HUB_TRIANGULATED_X = 4.537;

  public static final int BLUE_HUB_TOP_CENTER = 18;
  public static final int BLUE_HUB_TOP_OFFSET = 27;
  public static final int BLUE_HUB_BOTTOM_CENTER = 21;
  public static final int BLUE_HUB_BOTTOM_OFFSET = 24;
  public static final int BLUE_HUB_LEFT_TOP = 19;
  public static final int BLUE_HUB_LEFT_BOTTOM = 20;
  public static final int BLUE_HUB_RIGHT_TOP = 26;
  public static final int BLUE_HUB_RIGHT_BOTTOM = 25;
  public static final double BLUE_HUB_TRIANGULATED_X = 11.879;

  public static final double HUB_TRIANGULATED_Y = 4.035;
  public static final double HUB_TRIANGULATED_Z = 4.035;

  public static final int RED_TOWER_WALL_TOP = 16;
  public static final int RED_TOWER_WALL_BOTTOM = 15;

  public static final int RED_OUTPOST_TOP = 14;
  public static final int RED_OUTPOST_BOTTOM = 13;

  public static final int BLUE_TOWER_WALL_TOP = 31;
  public static final int BLUE_TOWER_WALL_BOTTOM = 32;

  public static final int BLUE_OUTPOST_TOP = 29;
  public static final int BLUE_OUTPOST_BOTTOM = 30;
}
