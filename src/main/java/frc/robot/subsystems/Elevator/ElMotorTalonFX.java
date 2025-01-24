package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Elevator.ElMotorIO.ElMotorIOInputs;
import frc.robot.util.Constants;

public class ElMotorTalonFX {

  private final TalonFX masterTalon = new TalonFX(Constants.elevatorConstants.masterID);
  private final TalonFX followerTalon = new TalonFX(Constants.elevatorConstants.followerID);
  private final CANcoder canCoder = new CANcoder(Constants.elevatorConstants.canCoderID);

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Torque-current control requests
  private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  // Inputs from drive motor
  private final StatusSignal<Angle> masterPosition = masterTalon.getPosition();
  private final StatusSignal<AngularVelocity> masterVelocity = masterTalon.getVelocity();
  private final StatusSignal<Voltage> masterAppliedVolts = masterTalon.getMotorVoltage();
  private final StatusSignal<Current> masterCurrent = masterTalon.getStatorCurrent();

  // Inputs from turn motor
  private final StatusSignal<Angle> followerAbsolutePosition = canCoder.getAbsolutePosition();
  private final StatusSignal<Angle> followerPosition = followerTalon.getPosition();
  private final StatusSignal<AngularVelocity> followerVelocity = followerTalon.getVelocity();
  private final StatusSignal<Voltage> followerAppliedVolts = followerTalon.getMotorVoltage();
  private final StatusSignal<Current> followerCurrent = followerTalon.getStatorCurrent();

  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

  private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  public ElMotorTalonFX() {
    TalonFXConfiguration config =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withClosedLoopRamps(
                new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(1.0))
            .withSlot0(new Slot0Configs().withKV(0.14).withKP(4.0).withKI(0).withKD(0));

    velocityVoltage.Slot = 0;

    this.masterTalon.getConfigurator().apply(config);
    this.followerTalon.getConfigurator().apply(config);

    this.followerTalon.setControl(new Follower(masterTalon.getDeviceID(), false));

    StatusSignal.setUpdateFrequencyForAll(
        50,
        masterAppliedVolts,
        masterCurrent,
        masterPosition,
        masterVelocity,
        followerAppliedVolts,
        followerCurrent,
        followerPosition,
        followerVelocity);
    this.masterTalon.optimizeBusUtilization();
    this.followerTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElMotorIOInputs inputs) {
    StatusSignal.refreshAll(
        masterAppliedVolts,
        masterCurrent,
        masterPosition,
        masterVelocity,
        followerAbsolutePosition,
        followerAppliedVolts,
        followerCurrent,
        followerPosition,
        followerVelocity);

    inputs.masterAppliedVolts = masterAppliedVolts.getValueAsDouble();
    inputs.masterCurrentAmps = masterCurrent.getValueAsDouble();
    inputs.masterPositionRad = masterPosition.getValueAsDouble();
    inputs.masterVelocityRadPerSec = masterVelocity.getValueAsDouble();

    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();
    inputs.followerPositionRad = followerPosition.getValueAsDouble();
    inputs.followerVelocityRadPerSec = followerVelocity.getValueAsDouble();

    inputs.extentionAbsPos = followerAbsolutePosition.getValueAsDouble();
  }

  @Override
  public void setElevatorVelocity(double velocityRotPerSecondLeft) {
    masterTalon.setControl(velocityVoltage.withVelocity(velocityRotPerSecondLeft));
  }

  @Override
  public void setElevatorPosition(double position){
    masterTalon.setPosition(position);
  }

}
