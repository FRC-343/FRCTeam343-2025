package frc.robot.subsystems.Intake;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.beambreak.BeambreakDigitalInput;
import frc.robot.beambreak.BeambreakIO;
import frc.robot.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.bobot_state2.BobotState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final BeambreakIO beambreak;
  // private final BeambreakIO beambreak2;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final BeambreakIOInputsAutoLogged beambreakInputs = new BeambreakIOInputsAutoLogged();
  // private final BeambreakIOInputsAutoLogged beambreak2Inputs = new BeambreakIOInputsAutoLogged();

  public Intake() {
    switch (Constants.currentMode) {
      case REAL:
        io = new IntakeIOTalonFX(22, false, 26);
        beambreak = new BeambreakDigitalInput(3);
        // beambreak2 = new BeambreakDigitalInput(8);

        break;
      case SIM:
        io = new IntakeIOSim(DCMotor.getKrakenX60(1), 3, 1, new PIDConstants(1, 0, 0));
        beambreak = new BeambreakDigitalInput(9);
        // beambreak2 = new BeambreakDigitalInput(8);
        break;
      case REPLAY:
      default:
        io = new IntakeIO() {};
        beambreak = new BeambreakIO() {};
        // beambreak2 = new BeambreakIO() {};

        break;
    }
  }

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    this.io.updateInputs(this.inputs);
    this.beambreak.updateInputs(this.beambreakInputs);
    // this.beambreak2.updateInputs(this.beambreakInputs);

    Logger.processInputs("Intake", this.inputs);
    Logger.processInputs("Intake/Beambreak", this.beambreakInputs);
    // Logger.processInputs("Intake/Beambreak2", this.beambreak2Inputs);

    // Make sure the motor actually stops when the robot disabled
    if (DriverStation.isDisabled()) {
      this.io.stop();
    }
  }

  public Command HPintake() {
    return new InstantCommand(() -> this.io.setPercentOutput(-.2), this)
        .until(BobotState.ElevatorBeam())
        .andThen(new WaitCommand(.3))
        .andThen(
            new InstantCommand(() -> this.io.setPercentOutput(-.1), this)
                .until(BobotState.ElevatorBeam().negate())
                .andThen(stopCommand()));
  }

  public Command intakewithstop() {
    return new RunCommand(() -> this.io.setPercentOutput(-.1), this)
        .until(BobotState.ElevatorBeam());
  }

  public Command setVelocityCommand(double velocityRotPerSecond) {
    return new InstantCommand(() -> this.io.setVelocity(velocityRotPerSecond), this);
  }

  public Command setVelocityThenStopCommand(double velocityRotPerSecond) {
    return new RunCommand(() -> this.io.setVelocity(velocityRotPerSecond), this)
        .finallyDo(io::stop);
  }

  public Command runForTime(double speed, double time) { // -.5 for out .5 for in
    return new RunCommand(() -> this.io.setPercentOutput(speed), this)
        .withTimeout(time)
        .andThen(io::stop);
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
    // playMusic();
    return new RunCommand(() -> this.io.setPercentOutput(percentDecimal), this).finallyDo(io::stop);
  }

  public Command setPercentOutputBeambreakCommand(double percentDecimal, Trigger test) {
    return new RunCommand(() -> this.io.setPercentOutput(percentDecimal), this)
        .onlyWhile(test)
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

  public void playMusic() {
    this.io.playMusic();
  }

  public void pauseMusic() {
    this.io.pauseMusic();
  }
}
