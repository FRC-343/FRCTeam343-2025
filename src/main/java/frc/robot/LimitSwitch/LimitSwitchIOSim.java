package frc.robot.LimitSwitch;

import edu.wpi.first.wpilibj.simulation.DIOSim;

public class LimitSwitchIOSim implements LimitSwitchIO {
  public final DIOSim LimitSwitchSim;

  public LimitSwitchIOSim(int dioChannel) {
    LimitSwitchSim = new DIOSim(dioChannel);
  }

  @Override
  public void updateInputs(LimitSwitchIOInputs inputs) {
    inputs.isObstructed = !LimitSwitchSim.getValue();
  }

  @Override
  public void overrideObstructed(boolean isObstructed) {
    this.LimitSwitchSim.setValue(!isObstructed);
  }
}
