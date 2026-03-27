package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
  public static Command toggleIntakeCommand(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.toggleIntake();
        },
        intake);
  }

  public static Command runIntakeCommand(Intake intake) {
    return Commands.run(
            () -> {
              intake.runIntake(0.9625);
            },
            intake)
        .finallyDo(
            () -> {
              intake.runIntake(0);
            });
  }

  public static Command runIntakeReverseCommand(Intake intake) {
    return Commands.run(
            () -> {
              intake.runIntake(-1);
            },
            intake)
        .finallyDo(
            () -> {
              intake.runIntake(0);
            });
  }

  public static Command incrementIntakeCommand(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.incrementIntake();
        },
        intake);
  }

  public static Command decrementIntakeCommand(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.decrementIntake();
        },
        intake);
  }
}
