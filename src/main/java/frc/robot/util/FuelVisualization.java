package frc.robot.util;

import static frc.robot.FieldConstants.BLUE_HUB_TRIANGULATED_X;
import static frc.robot.FieldConstants.HUB_TRIANGULATED_Y;
import static frc.robot.FieldConstants.HUB_TRIANGULATED_Z;
import static frc.robot.FieldConstants.RED_HUB_TRIANGULATED_X;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class FuelVisualization {
  private final Translation3d redHubLocation =
      new Translation3d(RED_HUB_TRIANGULATED_X, HUB_TRIANGULATED_Y, HUB_TRIANGULATED_Z);
  private final Translation3d blueHubLocation =
      new Translation3d(BLUE_HUB_TRIANGULATED_X, HUB_TRIANGULATED_Y, HUB_TRIANGULATED_Z);
  // transformation
  private static final Transform3d launcherTransform =
      new Transform3d(0.35, 0, 0.8, new Rotation3d(0.0, Units.degreesToRadians(-55.0), 0.0));

  private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();

  public static Command drawTrajectory() {
    return null;
  }
}
