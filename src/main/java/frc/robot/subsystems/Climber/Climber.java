package frc.robot.subsystems.Climber;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.LimitSwitch.LimitSwitchDigitalInput;
import frc.robot.LimitSwitch.LimitSwitchIO;
import frc.robot.LimitSwitch.LimitSwitchIOInputsAutoLogged;
import frc.robot.bobot_state2.BobotState;
import org.littletonrobotics.junction.Logger;

/*
 * self explanatory
 * Do note that out Visulizer is not currently working as of 2/6/2025
 */

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final LimitSwitchIO LimitSwitch;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final LimitSwitchIOInputsAutoLogged LimitSwitchInputs =
      new LimitSwitchIOInputsAutoLogged();
  private final PIDController pidController =
      new PIDController(
          0.5, // Replace with actual PID values when on the bot
          0, 0);

  private double setpointInches = 0.0;

  public Climber() {
    switch (Constants.currentMode) {
      case REAL:
        io = new ClimberIOTalonFX(24);
        LimitSwitch = new LimitSwitchDigitalInput(6);

        break;
      case SIM:
        io = new ClimberIOSim(DCMotor.getKrakenX60(2), 3, 1, new PIDConstants(1, 0, 0));
        LimitSwitch = new LimitSwitchDigitalInput(6);
        break;
      case REPLAY:
      default:
        io = new ClimberIO() {};
        LimitSwitch = new LimitSwitchDigitalInput(6);
        break;
    }
  }

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    this.LimitSwitch.updateInputs(this.LimitSwitchInputs);

    Logger.processInputs("Climber", this.inputs);
    Logger.processInputs("Climber/LimitSwitch", this.LimitSwitchInputs);

    if (DriverStation.isDisabled()) {
      this.setSetpoint(0.0);
      this.io.stop();
    }

    Logger.recordOutput("Climber/SetpointInches", setpointInches);

    // // Log Mechanisms
    // measuredVisualizer.update(this.inputs.extentionAbsPos);
    // setpointVisualizer.update(this.setpointInches);
    // // I'm not quite sure how this works, it is not working in sim.

    // BobotState.setElevatorUp(this.inputs.extentionAbsPos <= 1.0);
    BobotState.updateClimberState(this.LimitSwitchInputs.isObstructed);
  }

  public void reset() {
    // io.setElevatorPosition(0.0);
  }

  private void setSetpoint(double setpoint) {
    this.setpointInches = MathUtil.clamp(setpoint, 0, 56); // not real value
    this.pidController.setSetpoint(this.setpointInches);
  }

  public Command setSetpointCommand(double positionInches) {
    return new InstantCommand(() -> this.setSetpoint(positionInches));
  }

  public Command setSetpointCurrentCommand() {
    return new InstantCommand(() -> this.setSetpoint(this.inputs.extentionAbsPos));
  }

  public Command pidCommand() {
    return new RunCommand(
        () -> {
          double output = this.pidController.calculate(this.inputs.extentionAbsPos);
          setVoltage(output);
        },
        this);
  }

  public void setVoltage(double voltage) {
    this.io.setClimberVelocity(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  public Command Disengage() {
    return new InstantCommand(() -> this.io.engage());
  }

  public Command Engage() {
    return new InstantCommand(() -> this.io.disEngage());
  }

  // public Command setVolatageCommand(double voltage) {
  //   return new RunCommand(() -> this.io.setElevatorVelocity(voltage), this);
  // }

  // public Command setVelocityCommand(double velocityRotPerSecond) {
  //   return new InstantCommand(() -> this.io.setElevatorVelocity(velocityRotPerSecond), this);
  // }
  // public Trigger elevatorIsDown() {
  //   return new Trigger(
  //       () -> MathUtil.isNear(56, this.inputs.extentionAbsPos, 1.0));
  // }

  // public Trigger elevatorIsAtSubwooferShot() {
  //   return new Trigger(
  //       () ->
  //           MathUtil.isNear(
  //               ElevatorConstants.kSubwooferShotHeightInches, this.inputs.positionInches, 2.0));
  // }

  // public Trigger elevatorIsAtAmp() {
  //   return new Trigger(
  //       () ->
  //           MathUtil.isNear(
  //               ElevatorConstants.kAmpScoreHeightInches, this.inputs.positionInches, 1.0));
  // }

  // public Trigger elevatorIsAtTrap() {
  //   return new Trigger(
  //       () ->
  //           MathUtil.isNear(
  //               ElevatorConstants.kTrapScoreHeightInches, this.inputs.positionInches, 1.0));
  // }

  // public Trigger elevatorIsUp() {
  //   return new Trigger(
  //       () ->
  //           MathUtil.isNear(
  //               ElevatorConstants.kPivotClearanceHeightInches, this.inputs.positionInches, 1.0));
  // }

  // public void runPercentOutput(double percentDecimal) {
  //   double output =
  //       MetalUtils.percentWithSoftStops(
  //           percentDecimal,
  //           this.inputs.masterPositionRad + this.inputs.masterVelocityRadPerSec,
  //           0,
  //           0);
  //   this.io.setPercentOutput(output);
  // }

  // public Command runPercentOutputCommand(Double percentDecimal) {
  //   return new InstantCommand(() -> this.runPercentOutput(percentDecimal), this);
  // }

  public Trigger limitIsTriggered() {
    return new Trigger(() -> this.LimitSwitchInputs.isObstructed);
  }

  public Command setPercentOutputCommand(double velocityRotPerSecond) {
    return new RunCommand(() -> this.io.setPercentOutput(velocityRotPerSecond), this)
        .onlyIf(limitIsTriggered());
  }

  public Command runBack(double velocityRotPerSecond) {
    return new RunCommand(() -> this.io.setPercentOutput(velocityRotPerSecond), this);
  }

  public Command goForRotForward(double rot) {
    return new RunCommand(() -> this.io.goForRotForward(rot), this).onlyIf(limitIsTriggered());
  }

  public Command goForRotBack(double rot) {
    return new RunCommand(() -> this.io.goForRotBack(rot), this);
  }
}
