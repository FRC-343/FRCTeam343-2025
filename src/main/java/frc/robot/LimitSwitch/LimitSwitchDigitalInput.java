package frc.robot.LimitSwitch;

import edu.wpi.first.wpilibj.DigitalInput;

public class LimitSwitchDigitalInput implements LimitSwitchIO {
  private final DigitalInput LimitSwitch;

  public LimitSwitchDigitalInput(int dioChannel) {
    this.LimitSwitch = new DigitalInput(dioChannel);
  }

  @Override
  public void updateInputs(LimitSwitchIOInputs inputs) {
    inputs.isObstructed = !LimitSwitch.get();
  }
}
