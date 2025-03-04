package frc.robot.subsystems.Leds;

import org.littletonrobotics.junction.AutoLog;

public interface LEDIO {
  @AutoLog
  public static class LEDIOInputs {
    public String mode = "";
  }

  public default void updateInputs(LEDIOInputs inputs) {}

  public default void Idle() {}

  public default void Left() {}

  public default void Right() {}

  public default void notEngaged() {}

  public default void Engaged() {}

  public default void CoralClear() {}
}
