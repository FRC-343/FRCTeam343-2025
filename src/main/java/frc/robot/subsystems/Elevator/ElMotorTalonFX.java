package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Constants;

import java.util.Queue;

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


  public ElMotorTalonFX(int deviceID, boolean isInverted) {
    this.masterTalon.getConfigurator()
        .apply(new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        isInverted
                                ? InvertedValue.Clockwise_Positive
                                : InvertedValue.CounterClockwise_Positive))
                .withSlot0(new Slot0Configs()
                        .withKV(.12)
                        .withKP(.1)
                        .withKI(0)
                        .withKD(0)));
    velocityVoltage.Slot = 0;

    StatusSignal.setUpdateFrequencyForAll(50,
        masterPosition,
        masterVelocity,
        masterAppliedVolts,
        masterCurrent);
    masterTalon.optimizeBusUtilization();

    
  }

}
