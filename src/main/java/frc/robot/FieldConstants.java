package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class FieldConstants {
    public static final AprilTagFields FIELD_VERSION = AprilTagFields.k2026RebuiltAndymark;

    static AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(FIELD_VERSION);
}
