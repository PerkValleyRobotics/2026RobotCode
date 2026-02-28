package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();
  private HopperIO io;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void runHopper(double speed) {
    io.setHopperSpeed(speed);
  }

  public void stopHopper() {
    io.setHopperSpeed(0);
  }
}
