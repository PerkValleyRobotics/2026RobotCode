package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommands {
  public static Command toggleIntake(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.toggleIntake();
        },
        intake);
  }

  public static Command runIntake(Intake intake) {
    return Commands.run(
            () -> {
              intake.runIntake(1);
            },
            intake)
        .finallyDo(
            () -> {
              intake.runIntake(0);
            });
  }

public static Command runIntakeReverse(Intake intake) {
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

  public static Command incrementIntake(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.incrementIntake();
        },
        intake);
  }

  public static Command decrementIntake(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.decrementIntake();
        },
        intake);
  }
}
