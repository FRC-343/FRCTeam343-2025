package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bobot_state.BobotState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.MetalUtils;

public class Elevator extends SubsystemBase {
  private final ElMotorIO io;

  private final ElMotorIOInputsAutoLogged inputs = new ElMotorIOInputsAutoLogged();

  private final PIDController pidController =
      new PIDController(
        0,              // Replace with actual PID values when on the bot
         0, 
         0);

  private double setpointInches = 0.0;

  private final ElevatorVisualizer measuredVisualizer =
      new ElevatorVisualizer("Measured", Color.kBlack);
  private final ElevatorVisualizer setpointVisualizer =
      new ElevatorVisualizer("Setpoint", Color.kGreen);


  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);

    Logger.processInputs("Elevator", this.inputs);

    if (DriverStation.isDisabled()) {
      this.setSetpoint(0.0);
      this.io.stop();
    }

    Logger.recordOutput("Elevator/SetpointInches", setpointInches);

    // Log Mechanisms
    measuredVisualizer.update(this.inputs.extentionAbsPos);
    setpointVisualizer.update(this.setpointInches);

    BobotState.setElevatorUp(this.inputs.extentionAbsPos <= 1.0);
  }

  public void reset() {
    io.setElevatorPosition(0.0);
  }

  private void setSetpoint(double setpoint) {
    this.setpointInches =
        MathUtil.clamp(
            setpoint, 0, 56);
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
    this.io.setElevatorVelocity(MathUtil.clamp(voltage, -12.0, 12.0));
  }

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

  public void runPercentOutput(double percentDecimal) {
    double output =
        MetalUtils.percentWithSoftStops(
            percentDecimal,
            this.inputs.extentionAbsPos + this.inputs.masterVelocityRadPerSec,
            0,
            0);
    this.io.setPercentOutput(output);
  }

  public Command runPercentOutputCommand(DoubleSupplier percentDecimal) {
    return new RunCommand(() -> this.runPercentOutput(percentDecimal.getAsDouble()), this);
  }

}

