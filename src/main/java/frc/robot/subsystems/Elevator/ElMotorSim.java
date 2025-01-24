package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElMotorSim implements ElMotorIO {
  private static final double kInchesPerRotation = 2 * Math.PI;

  private static final double momentOfInertiaKgMSquared =
      0.05; // Moment of intertia (totally wrong)

  private final DCMotorSim sim =
      new DCMotorSim(null, DCMotor.getKrakenX60(2), momentOfInertiaKgMSquared, 5);

  private double appliedVoltage = 0.0;

  public void updateInputs(ElMotorIOInputs inputs) {
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
}
