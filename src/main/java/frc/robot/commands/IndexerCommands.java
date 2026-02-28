package frc.robot.commands;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;

public class IndexerCommands {
  public static Command runIndexer(Indexer launcher) {
    return Commands.run(
            () -> {
              launcher.runIndexer(INDEXER_MOTOR_SPEED);
            },
            launcher)
        .finallyDo(
            () -> {
              launcher.stopIndexer();
            });
  }
  ;
}
