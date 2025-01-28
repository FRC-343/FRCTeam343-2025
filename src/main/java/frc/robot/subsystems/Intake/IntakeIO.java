package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double positionRad = 0.0;
    public double velocityRPS = 0.0;
    public double appliedVolts = 0.0;
    public double appliedDutyCycle = 0.0;
    public double[] currentAmps = new double[] {};
    public boolean DetectedNote;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void DetectedNote(Boolean Noted) {}

  /** Run open loop at the specified voltage. */
  public default void setSpeed(double Speed) {}

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(double velocityRPS) {}

  /** Stop in open loop. */
  public default void stop() {}

  public default void setPercentOutput(double percentDecimal) {}

  public default void setVoltage(double voltage) {}
  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
