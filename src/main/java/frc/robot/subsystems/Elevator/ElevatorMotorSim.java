package frc.robot.subsystems.Elevator;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorMotorSim implements ElevatorMotorIO {
  private static final double kInchesPerRotation = 2 * Math.PI;

  private static final double momentOfInertiaKgMSquared =
      0.05; // Moment of intertia (totally wrong)

  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;
  private final PIDController controller;

  public ElevatorMotorSim(
      DCMotor motorModel, double reduction, double moi, PIDConstants pidConstants) {
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
    controller = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
  }

  public void updateInputs(ElevatorMotorIOInputs inputs) {
    sim.update(0.02);

    inputs.masterAppliedVolts = appliedVoltage;
    inputs.masterCurrentAmps = sim.getCurrentDrawAmps();
    inputs.masterVelocityRadPerSec = sim.getAngularVelocityRPM() / 60 * kInchesPerRotation;
    inputs.masterPositionRad = sim.getAngularPositionRotations() * kInchesPerRotation;
  }

  @Override
  public void setElevatorVelocity(double voltage) {
    appliedVoltage = voltage;
    sim.setInputVoltage(voltage);
  }

  @Override
  public void setPercentOutput(double percentDecimal) {
    // sim.setState((percentDecimal * 1000) / kInchesPerRotation, sim.getAngularVelocityRadPerSec());
    sim.setInputVoltage(percentDecimal);
  }

  @Override
  public void setElevatorPosition(double rotation) {
    sim.setAngle(rotation);
  }
}
