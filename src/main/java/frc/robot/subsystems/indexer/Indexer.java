// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private IndexerIO io;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void runIndexer(double speed) {
    io.setIndexerSpeed(speed);
  }

  public void stopIndexer() {
    io.setIndexerSpeed(0);
  }
}
