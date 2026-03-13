// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.hopper.HopperConstants.HOPPER_NORMAL_SPEED;
import static frc.robot.subsystems.hopper.HopperConstants.INDEXER_NORMAL_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;

public class HopperCommands {
  public static Command runHopper(Hopper hopper) {
    return Commands.run(
            () -> {
              hopper.runHopper(HOPPER_NORMAL_SPEED);
            },
            hopper)
        .finallyDo(
            () -> {
              hopper.stopHopper();
            });
  }

  public static Command runIndexer(Hopper hopper) {
    return Commands.run(
            () -> {
              hopper.runIndexer(INDEXER_NORMAL_SPEED);
            },
            hopper)
        .finallyDo(
            () -> {
              hopper.stopIndexer();
            });
  }
}
