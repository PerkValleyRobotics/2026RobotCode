package frc.robot.commands;

import static frc.robot.subsystems.launcher.LauncherConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.launcher.Launcher;

public class LauncherCommands {
  public static Command runLauncher(Launcher launcher) {
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
  ;
}
