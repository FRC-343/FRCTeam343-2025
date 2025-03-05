package frc.robot.subsystems.Leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Leds.LEDIO.LEDIOInputs;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class LEDInput extends VirtualSubsystem {

  private final LEDIO io;

  private final LEDIOInputs inputs = new LEDIOInputs();

  public LEDInput() {
    switch (Constants.currentMode) {
      case REAL:
        io = new LEDIO() {};
        break;
      case SIM:
        io = new LEDIO() {};
        break;
      case REPLAY:
      default:
        io = new LEDIO() {};
        break;
    }
  }

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);

    Logger.recordOutput("LED/Mode", inputs.mode);
  }

  public Command Idle() {
    return new InstantCommand(() -> this.io.Idle());
  }

  public Command Left() {
    return new InstantCommand(() -> this.io.Left());
  }

  public Command Right() {
    return new RunCommand(() -> this.io.Right());
  }

  @Override
  public void simulationPeriodic() {
    this.io.updateInputs(this.inputs);

    Logger.recordOutput("LED/Mode", inputs.mode);
  }
}
