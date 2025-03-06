package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIOInputs;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX talon;
  private final TalonFX follower = new TalonFX(20);

  // private final SparkBase encoder = new SparkMax(26, MotorType.kBrushed);
  // private final AbsoluteEncoder absEnc;

  private final Servo Servo = new Servo(1);

  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Double> dutyCycle;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Angle> position;

  private final StatusSignal<Voltage> followerVoltage = follower.getMotorVoltage();
  private final StatusSignal<Double> followerDutyCycle = follower.getDutyCycle();
  private final StatusSignal<AngularVelocity> followerVelocity = follower.getVelocity();
  private final StatusSignal<Angle> followerPosition = follower.getPosition();

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

  private final MotionMagicVoltage Vrequest = new MotionMagicVoltage(0);

  public ClimberIOTalonFX(int deviceId) {
    talon = new TalonFX(deviceId);
    voltage = talon.getMotorVoltage();
    dutyCycle = talon.getDutyCycle();
    velocity = talon.getVelocity();
    position = talon.getPosition();

    // absEnc = encoder.getAbsoluteEncoder();

    talon
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(new Slot0Configs().withKV(0.12).withKP(1).withKI(0).withKD(0))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicAcceleration(2)
                        .withMotionMagicCruiseVelocity(5)
                        .withMotionMagicJerk(5)));
    velocityVoltage.Slot = 0;

    this.follower
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withSlot0(new Slot0Configs().withKV(0.12).withKP(1).withKI(0).withKD(0))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicAcceleration(2)
                        .withMotionMagicCruiseVelocity(5)
                        .withMotionMagicJerk(5)));
    velocityVoltage.Slot = 0;

    StatusSignal.setUpdateFrequencyForAll(
        10,
        voltage,
        dutyCycle,
        velocity,
        position,
        followerDutyCycle,
        followerPosition,
        followerVelocity,
        followerVoltage);
    talon.optimizeBusUtilization();
    this.follower.optimizeBusUtilization();
  }

  public void updateInputs(ClimberIOInputs inputs) {
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
    inputs.masterVelocityRadPerSec = velocity.getValueAsDouble() / 3.0;
    inputs.masterPositionRad = position.getValueAsDouble();

    inputs.followerAppliedVolts = followerVoltage.getValueAsDouble();
    inputs.followerVelocityRadPerSec = followerVelocity.getValueAsDouble();
    inputs.followerPositionRad = followerPosition.getValueAsDouble();

    // inputs.extentionAbsPos = absEnc.getPosition();
  }

  @Override
  public void setClimberVelocity(double velocityRotPerSecond) {
    talon.setControl(dutyCycleOut.withOutput(velocityRotPerSecond));
    this.follower.setControl(dutyCycleOut.withOutput(-velocityRotPerSecond));
  }

  @Override
  public void setPercentOutput(double percentDecimal) {
    talon.setControl(dutyCycleOut.withOutput(percentDecimal));
    this.follower.setControl(dutyCycleOut.withOutput(-percentDecimal));
  }

  @Override
  public void setSetpoint(double setpoint) {
    talon.setControl(dutyCycleOut.withOutput(setpoint));
    this.follower.setControl(dutyCycleOut.withOutput(-setpoint));
  }

  @Override
  public void setVoltage(double voltage) {
    talon.setControl(dutyCycleOut.withOutput(voltage));
  }

  @Override
  public void disEngage() {
    this.Servo.set(-1);
    this.Servo.setAngle(0);
  }

  @Override
  public void engage() {
    this.Servo.set(1);
    this.Servo.setAngle(260);
  }

  @Override
  public void goForRotForward(double voltage) {
    talon.setControl(Vrequest.withPosition(voltage));
    this.follower.setControl(Vrequest.withPosition(-voltage));
  }

  @Override
  public void goForRotBack(double voltage) {
    talon.setControl(Vrequest.withPosition(voltage));
    this.follower.setControl(Vrequest.withPosition(-voltage));
  }
}
