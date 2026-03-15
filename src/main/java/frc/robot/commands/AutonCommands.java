// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.launcher.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonCommands extends SequentialCommandGroup {
  /** Creates a new shoot. */
  public AutonCommands(Launcher launcher, Hopper hopper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        LauncherCommands.startLauncher(launcher),
        Commands.waitSeconds(4),
        HopperCommands.runHopper(hopper),
        HopperCommands.runIndexer(hopper));
  }
}
