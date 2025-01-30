package frc.robot.subsystems.intake;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.beambreak.BeambreakDigitalInput;
import frc.robot.beambreak.BeambreakIO;
import frc.robot.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.beambreak.BeambreakIOSim;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final BeambreakIO beambreak;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final BeambreakIOInputsAutoLogged beambreakInputs = new BeambreakIOInputsAutoLogged();

  public Intake() {
    switch (Constants.currentMode) {
      case REAL:
        io = new IntakeIOTalonFX(21, false);
        beambreak = new BeambreakDigitalInput(0);
        break;
      case SIM:
        io = new IntakeIOSim(DCMotor.getKrakenX60(21), 3, 1, new PIDConstants(1, 0, 0));
        beambreak = new BeambreakIOSim(0);
        break;
      case REPLAY:
      default:
        io = new IntakeIO() {};
        beambreak = new BeambreakIO() {};

        break;
    }
  }

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    this.io.updateInputs(this.inputs);
    this.beambreak.updateInputs(this.beambreakInputs);

    Logger.processInputs("Intake", this.inputs);
    Logger.processInputs("Intake/Beambreak", this.beambreakInputs);

    // Make sure the motor actually stops when the robot disabled
    if (DriverStation.isDisabled()) {
      this.io.stop();
    }
  }

  public Command setVelocityCommand(double velocityRotPerSecond) {
    return new InstantCommand(() -> this.io.setVelocity(velocityRotPerSecond), this);
  }

  public Command setVelocityThenStopCommand(double velocityRotPerSecond) {
    return new RunCommand(() -> this.io.setVelocity(velocityRotPerSecond), this)
        .finallyDo(io::stop);
  }

  public Command setVelocityBeambreakCommand(double velocityRotPerSecond) {
    return new RunCommand(() -> this.io.setVelocity(velocityRotPerSecond), this)
        .unless(beambreakIsObstructed())
        .until(beambreakIsObstructed())
        .andThen(stopCommand());
  }

  public Command setPercentOutputCommand(double velocityRotPerSecond) {
    return new InstantCommand(() -> this.io.setPercentOutput(velocityRotPerSecond), this);
  }

  public Command setPercentOutputThenStopCommand(double percentDecimal) {
    return new RunCommand(() -> this.io.setPercentOutput(percentDecimal), this).finallyDo(io::stop);
  }

  public Command setPercentOutputBeambreakCommand(double percentDecimal) {
    return new RunCommand(() -> this.io.setPercentOutput(percentDecimal), this)
        .unless(beambreakIsObstructed())
        .until(beambreakIsObstructed())
        .andThen(stopCommand());
  }

  public Command stopCommand() {
    return new InstantCommand(this.io::stop, this);
  }

  // For testing and sim
  public Command setBeambreakObstructedCommand(boolean value) {
    return new InstantCommand(
        () -> {
          this.beambreak.overrideObstructed(value);
        });
  }

  // For testing and sim
  public Command toggleBeambreakObstructedCommand() {
    return new InstantCommand(
        () -> {
          this.beambreak.overrideObstructed(!this.beambreakInputs.isObstructed);
        });
  }

  public Trigger beambreakIsObstructed() {
    return new Trigger(() -> this.beambreakInputs.isObstructed);
  }
}
