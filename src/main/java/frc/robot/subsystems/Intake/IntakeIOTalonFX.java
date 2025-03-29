package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX talon;
  private final TalonFX talon2;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Double> dutyCycle;
  private final StatusSignal<AngularVelocity> velocity;

  private final StatusSignal<Voltage> voltage2;
  private final StatusSignal<Double> dutyCycle2;
  private final StatusSignal<AngularVelocity> velocity2;

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final DutyCycleOut dutyCycleOut2 = new DutyCycleOut(0);

  private final Orchestra m_orchestra = new Orchestra();

  public IntakeIOTalonFX(int deviceId, boolean isInverted, int deviceId2) {
    talon = new TalonFX(deviceId);
    talon2 = new TalonFX(deviceId2);
    voltage = talon.getMotorVoltage();
    dutyCycle = talon.getDutyCycle();
    velocity = talon.getVelocity();

    voltage2 = talon.getMotorVoltage();
    dutyCycle2 = talon.getDutyCycle();
    velocity2 = talon.getVelocity();

    this.m_orchestra.addInstrument(talon);
    this.m_orchestra.loadMusic("output2.chrp");

    talon
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(
                            isInverted
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive))
                .withSlot0(new Slot0Configs().withKV(0.12).withKP(1).withKI(0).withKD(0)));
    velocityVoltage.Slot = 0;

    talon2
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(
                            isInverted
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive))
                .withSlot0(new Slot0Configs().withKV(0.12).withKP(1).withKI(0).withKD(0)));
    velocityVoltage.Slot = 0;

    StatusSignal.setUpdateFrequencyForAll(10, voltage, dutyCycle, velocity);
    talon.optimizeBusUtilization();
    talon2.optimizeBusUtilization();

    // talon2.setControl(new Follower(talon.getDeviceID(), true));
  }

  public void updateInputs(IntakeIOInputs inputs) {
    StatusSignal.refreshAll(velocity, dutyCycle, voltage);
    inputs.appliedVoltage = voltage.getValueAsDouble();
    inputs.appliedDutyCycle = dutyCycle.getValueAsDouble();
    inputs.velocityRotPerSecond = velocity.getValueAsDouble() / 3.0;

    StatusSignal.refreshAll(velocity, dutyCycle, voltage);
    inputs.appliedVoltage2 = voltage2.getValueAsDouble();
    inputs.appliedDutyCycle2 = dutyCycle2.getValueAsDouble();
    inputs.velocityRotPerSecond2 = velocity2.getValueAsDouble() / 3.0;
  }

  @Override
  public void setVelocity(double velocityRotPerSecond) {
    talon.setControl(velocityVoltage.withVelocity(velocityRotPerSecond * 3.0));
  }

  @Override
  public void setPercentOutput(double percentDecimal) {
    talon.setControl(dutyCycleOut.withOutput(percentDecimal));
    talon2.setControl(dutyCycleOut.withOutput(-(percentDecimal * 2)));
  }

  @Override
  public void setPercentOutputT2(double percentDecimal) {
    talon2.setControl(dutyCycleOut.withOutput(-percentDecimal));
  }

  @Override
  public void setPercentOutputT1(double percentDecimal) {
    talon.setControl(dutyCycleOut.withOutput(percentDecimal));
  }

  @Override
  public void playMusic() {
    m_orchestra.play();
  }

  @Override
  public void pauseMusic() {
    m_orchestra.pause();
  }

  @Override
  public void setVoltage(double voltage) {
    talon.setVoltage(voltage);
    talon2.setVoltage(voltage * 2);
  }
}
