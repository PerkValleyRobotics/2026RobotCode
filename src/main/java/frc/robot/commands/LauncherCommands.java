package frc.robot.commands;

import static frc.robot.subsystems.launcher.LauncherConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.subsystems.trajectoryCalc.LauncherTrajectoryCalc.inputParameters;

public class LauncherCommands {
  public static Command runLauncherCommand(Launcher launcher) {
    return Commands.run(
            () -> {
              launcher.runLauncher(SHOOTING_MOTOR_SPEED);
            },
            launcher)
        .finallyDo(
            () -> {
              launcher.stopLauncher();
            });
  }

  public static Command startLauncherCommand(Launcher launcher) {
    return Commands.run(
        () -> {
          launcher.runLauncher(SHOOTING_MOTOR_SPEED);
        });
  }

  public static Command lockTrajectoryCommand(Launcher launcher, Drive drive) {
    return Commands.runOnce(
        () -> {
          launcher.calculateAndSetTrajectoryAngle(
              new inputParameters(7.5, drive.getPose(), new Translation2d(0, 0)));
        });
  }
}
