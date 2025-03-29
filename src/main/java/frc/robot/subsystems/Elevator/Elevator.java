package frc.robot.subsystems.Elevator;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.LimitSwitch.LimitSwitchDigitalInput;
import frc.robot.LimitSwitch.LimitSwitchIO;
import frc.robot.LimitSwitch.LimitSwitchIOInputsAutoLogged;
import frc.robot.beambreak.BeambreakDigitalInput;
import frc.robot.beambreak.BeambreakIO;
import frc.robot.beambreak.BeambreakIOInputsAutoLogged;
import frc.robot.bobot_state2.BobotState;
import frc.robot.util.Constant;
import org.littletonrobotics.junction.Logger;

/*
 * self explanatory
 * Do note that out Visulizer is not currently working as of 2/6/2025
 */

public class Elevator extends SubsystemBase {
  private final ElevatorMotorIO io;
  private final BeambreakIO beambreak;
  private final LimitSwitchIO LimitSwitch;
  private final LimitSwitchIO LimitSwitchBackup;

  boolean test = false;

  private final ElevatorMotorIOInputsAutoLogged inputs = new ElevatorMotorIOInputsAutoLogged();
  private final BeambreakIOInputsAutoLogged beambreakInputs = new BeambreakIOInputsAutoLogged();
  private final LimitSwitchIOInputsAutoLogged LimitSwitchInputs =
      new LimitSwitchIOInputsAutoLogged();
  private final LimitSwitchIOInputsAutoLogged LimitSwitchBackupInputs =
      new LimitSwitchIOInputsAutoLogged();

  private final PIDController pidController =
      new PIDController(
          0.5, // Replace with actual PID values when on the bot
          0, 0);

  private final ElevatorVisualizer measuredVisualizer =
      new ElevatorVisualizer("Measured", Color.kBlack);
  private final ElevatorVisualizer setpointVisualizer =
      new ElevatorVisualizer("Setpoint", Color.kGreen);

  private double setpointInches = 0.0;

  public Elevator() {
    switch (Constants.currentMode) {
      case REAL:
        io = new ElevatorMotorTalonFX(21);
        beambreak = new BeambreakDigitalInput(9); // 3 and 2
        LimitSwitch = new LimitSwitchDigitalInput(0);
        LimitSwitchBackup = new LimitSwitchDigitalInput(1);

        break;
      case SIM:
        io = new ElevatorMotorSim(DCMotor.getKrakenX60(1), 3, 1, new PIDConstants(1, 0, 0));
        beambreak = new BeambreakDigitalInput(2);
        LimitSwitch = new LimitSwitchDigitalInput(0);
        LimitSwitchBackup = new LimitSwitchDigitalInput(1);

        break;
      case REPLAY:
      default:
        io = new ElevatorMotorIO() {};
        beambreak = new BeambreakIO() {};
        LimitSwitch = new LimitSwitchIO() {};
        LimitSwitchBackup = new LimitSwitchIO() {};
        break;
    }
  }

  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    this.beambreak.updateInputs(this.beambreakInputs);
    this.LimitSwitch.updateInputs(this.LimitSwitchInputs);
    this.LimitSwitchBackup.updateInputs(this.LimitSwitchBackupInputs);

    Logger.processInputs("Elevator", this.inputs);

    if (DriverStation.isDisabled()) {
      this.setSetpoint(0.0);
      this.io.stop();
    }

    Logger.processInputs("Elevator/Beambreak", this.beambreakInputs);
    Logger.processInputs("Elevator/LimitSwitch", this.LimitSwitchInputs);
    Logger.processInputs("Elevator/LimitSwitchBackup", this.LimitSwitchBackupInputs);

    Logger.recordOutput("Elevator/SetpointInches", setpointInches);

    // Log Mechanisms
    // measuredVisualizer.update(this.inputs.masterPositionRad);
    // setpointVisualizer.update(this.setpointInches);
    // // I'm not quite sure how this works, it is semi working in sim.

    // resetEncoder();

    // BobotState.setElevatorUp(this.inputs.masterPositionRad >= 1.0);

    BobotState.updateElevatorBeam(beambreakIsObstructed().getAsBoolean());
    BobotState.updateElevatorState(higherThanMid().getAsBoolean());
    BobotState.updateElevatorPose(this.inputs.masterPositionRad);

    // limitIsTriggered().onTrue(resetEncoder());
    // BackupLimitIsTriggerd().onTrue(resetEncoder());
    // elevatorIsDown().onFalse(resetEncoder());
  }

  // These needs to be reorganized

  public Command overrideBeambreakObstructedCommand(boolean value) {
    return new InstantCommand(
        () -> {
          this.beambreak.overrideObstructed(value);
        });
  }

  private void setSetpoint(double setpoint) {
    setpointInches = MathUtil.clamp(setpoint, 0, 56); // not real value
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

  public Command setElevatorPosition(double position) {
    return new RunCommand(() -> this.io.setElevatorPosition(position))
        .unless(beambreakIsObstructed().and(elevatorIsDown()));
  }

  public Command setElevatorPositionL4() {
    System.out.println(elevatorNearL4().getAsBoolean());

    return new RunCommand(
            () -> this.io.setElevatorPosition(Constant.elevatorConstants.L4Level), this)
        .until(elevatorNearL4());
  }

  public Command setElevatorPositionL3() {
    System.out.println(elevatorNearL3().getAsBoolean());

    return new RunCommand(
            () -> this.io.setElevatorPosition(Constant.elevatorConstants.L3Level), this)
        .until(elevatorNearL3());
  }

  public Command setElevatorPositionL2() {
    System.out.println(elevatorNearL2().getAsBoolean());

    return new RunCommand(
            () -> this.io.setElevatorPosition(Constant.elevatorConstants.L2Level), this)
        .until(elevatorNearL2());
  }

  public Command setElevatorPositionFeed() {
    System.out.println(elevatorIsDown().getAsBoolean());

    return new RunCommand(() -> this.io.setElevatorPosition(Constant.elevatorConstants.FEED), this)
        .until(elevatorIsDown());
  }

  public Trigger elevatorNearL4() {
    return new Trigger(
        () ->
            MathUtil.isNear(Constant.elevatorConstants.L4Level, this.inputs.masterPositionRad, 1));
  }

  public Trigger elevatorNearL3() {
    return new Trigger(
        () ->
            MathUtil.isNear(Constant.elevatorConstants.L3Level, this.inputs.masterPositionRad, 1));
  }

  public Trigger elevatorNearL2() {
    return new Trigger(
        () ->
            MathUtil.isNear(Constant.elevatorConstants.L2Level, this.inputs.masterPositionRad, 1));
  }

  public Trigger higherThanMid() {
    return new Trigger(() -> (this.inputs.masterPositionRad > 20));
  }

  public Command stopCommand() {
    return new InstantCommand(this.io::stop, this);
  }

  public void setVoltage(double voltage) {
    this.io.setElevatorVelocity(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  public Command setVolatageCommand(double voltage) {
    return new RunCommand(() -> this.io.setElevatorVelocity(voltage), this);
  }

  public Command resetEncoder() {
    return new InstantCommand(this.io::resetEncoder, this);
  }

  public Trigger beambreakIsObstructed() {
    return new Trigger(() -> this.beambreakInputs.isObstructed);
  }

  public Trigger limitIsTriggered() {
    return new Trigger(() -> this.LimitSwitchInputs.isObstructed);
  }

  public Trigger BackupLimitIsTriggerd() {
    return new Trigger(() -> this.LimitSwitchBackupInputs.isObstructed);
  }

  public Trigger elevatorIsDown() {
    return new Trigger(() -> MathUtil.isNear(0, this.inputs.masterPositionRad, 1));
  }

  public Command setPercentOutputCommand(double velocityRotPerSecond) {
    setpointInches = velocityRotPerSecond * 1000;
    return new RunCommand(() -> this.io.setPercentOutput(velocityRotPerSecond), this);
  }

  /* Adding new commands down here to ease readability,
   * at some point the above commands will be reorganized.
   * Triggers might also be separated at a later date, potentially added to BobotState
   */

}
