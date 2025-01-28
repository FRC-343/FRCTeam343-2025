package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io = new IntakeIO() {};
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final DigitalInput NoteDetector = new DigitalInput(8);

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    Logger.processInputs("Intake", inputs);
    // if (getNoteDetector() == false) {
    //   stop();
    // }
  }

  public boolean getNoteDetector() {
    return NoteDetector.get(); // false = note in intake
  }

  /** Run open loop at the specified voltage. */
  public void runSpeed(double Speed) {
    if (getNoteDetector() == true) {
      io.setSpeed(Speed);
    }
  }

  public void runBypass(double speed) {
    io.setSpeed(speed);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRPM) {
    var velocityRadPerSec = velocityRPM;
    io.setVelocity(velocityRadPerSec);

    // Log Intake setpoint
    Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
  }

  public boolean DetectedNote(boolean Noted) {
    return getNoteDetector();
  }

  /** Stops the Intake. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RPM. */
  @AutoLogOutput
  public double getVelocityRPM() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRPS);
  }

  /** Returns the current velocity in radians per second. */
  public double getCharacterizationVelocity() {
    return inputs.velocityRPS;
  }
}
