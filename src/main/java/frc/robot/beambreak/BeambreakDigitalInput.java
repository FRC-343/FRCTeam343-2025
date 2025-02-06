package frc.robot.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeambreakDigitalInput implements BeambreakIO {
  private final DigitalInput beambreak;

  public BeambreakDigitalInput(int dioChannel) {
    this.beambreak = new DigitalInput(dioChannel);
  }

  @Override
  public void updateInputs(BeambreakIOInputs inputs) {
    inputs.isObstructed = !beambreak.get();
  }
}

