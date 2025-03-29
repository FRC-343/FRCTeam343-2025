package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Elevator.ElevatorMotorIO.ElevatorMotorIOInputs;

public class ElevatorMotorTalonFX implements ElevatorMotorIO {
  private final TalonFX talon;
  private final TalonFX follower = new TalonFX(23);

  // private final SparkBase encoder = new SparkMax(25, null);
  // private final AbsoluteEncoder absEnc;

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Double> dutyCycle;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Angle> position;
  private final StatusSignal<Current> current;

  private final StatusSignal<Voltage> followerVoltage = follower.getMotorVoltage();
  private final StatusSignal<Double> followerDutyCycle = follower.getDutyCycle();
  private final StatusSignal<AngularVelocity> followerVelocity = follower.getVelocity();
  private final StatusSignal<Angle> followerPosition = follower.getPosition();
  private final StatusSignal<Current> followerCurrent = follower.getStatorCurrent();

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private final MotionMagicVoltage Vrequest = new MotionMagicVoltage(0);

  private final Orchestra m_orchestra = new Orchestra();

  public ElevatorMotorTalonFX(int deviceId) {
    talon = new TalonFX(deviceId);
    voltage = talon.getMotorVoltage();
    dutyCycle = talon.getDutyCycle();
    velocity = talon.getVelocity();
    position = talon.getPosition();
    current = talon.getStatorCurrent();

    // absEnc = encoder.getAbsoluteEncoder();

    this.m_orchestra.addInstrument(talon);
    this.m_orchestra.addInstrument(this.follower);
    this.m_orchestra.loadMusic("output.chrp");

    talon
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(
                    new Slot0Configs().withKV(0.12).withKA(.01).withKP(2).withKI(0).withKD(0))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicAcceleration(25)
                        .withMotionMagicCruiseVelocity(25)
                        .withMotionMagicJerk(200)));
    velocityVoltage.Slot = 0;

    this.follower
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(
                    new Slot0Configs().withKV(0.12).withKA(.01).withKP(2).withKI(0).withKD(0))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicAcceleration(25)
                        .withMotionMagicCruiseVelocity(25)
                        .withMotionMagicJerk(200)));
    velocityVoltage.Slot = 0;

    StatusSignal.setUpdateFrequencyForAll(
        10,
        voltage,
        dutyCycle,
        velocity,
        position,
        current,
        followerDutyCycle,
        followerPosition,
        followerVelocity,
        followerVoltage,
        followerCurrent);
    talon.optimizeBusUtilization();
    this.follower.optimizeBusUtilization();

    this.follower.setControl(new Follower(talon.getDeviceID(), false));
  }

  public void updateInputs(ElevatorMotorIOInputs inputs) {
    StatusSignal.refreshAll(
        velocity,
        dutyCycle,
        voltage,
        position,
        followerDutyCycle,
        followerPosition,
        followerVelocity,
        followerVoltage);
    inputs.masterAppliedVolts = voltage.getValueAsDouble();
    inputs.masterVelocityRadPerSec = velocity.getValueAsDouble();
    inputs.masterPositionRad = position.getValueAsDouble();
    inputs.masterCurrentAmps = current.getValueAsDouble();

    inputs.followerAppliedVolts = followerVoltage.getValueAsDouble();
    inputs.followerVelocityRadPerSec = followerVelocity.getValueAsDouble();
    inputs.followerPositionRad = followerPosition.getValueAsDouble();
    inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();

    // inputs.extentionAbsPos = absEnc.getPosition();
  }

  @Override
  public void setElevatorVelocity(double velocityRotPerSecond) {
    talon.setControl(dutyCycleOut.withOutput(velocityRotPerSecond));
    // this.follower.setControl(dutyCycleOut.withOutput(velocityRotPerSecond));
  }

  @Override
  public void setPercentOutput(double percentDecimal) {
    talon.setControl(dutyCycleOut.withOutput(percentDecimal));
    // this.follower.setControl(dutyCycleOut.withOutput(percentDecimal));
  }

  @Override
  public void setSetpoint(double setpoint) {
    talon.setControl(dutyCycleOut.withOutput(setpoint));
    // this.follower.setControl(dutyCycleOut.withOutput(setpoint));
  }

  @Override
  public void setVoltage(double voltage) {
    talon.setControl(Vrequest.withPosition(voltage));
  }

  @Override
  public void setElevatorPosition(double rotation) {
    talon.setControl(Vrequest.withPosition(rotation));
  }

  @Override
  public void resetEncoder() {
    talon.setPosition(0);
  }
}
