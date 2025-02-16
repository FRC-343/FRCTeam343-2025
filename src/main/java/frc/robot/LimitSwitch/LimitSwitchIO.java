package frc.robot.LimitSwitch;

import org.littletonrobotics.junction.AutoLog;

public interface LimitSwitchIO {
  @AutoLog
  public static class LimitSwitchIOInputs {
    public boolean isObstructed = false;
  }

  public default void updateInputs(LimitSwitchIOInputs inputs) {}

  public default void overrideObstructed(boolean isObstructed) {}
}
