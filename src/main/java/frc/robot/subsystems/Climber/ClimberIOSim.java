package frc.robot.subsystems.Climber;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
  private static final double kInchesPerRotation = 2 * Math.PI;

  private static final double momentOfInertiaKgMSquared =
      0.05; // Moment of intertia (totally wrong)

  private final DCMotorSim sim;
  private double appliedVoltage = 0.0;
  private final PIDController controller;

  public ClimberIOSim(DCMotor motorModel, double reduction, double moi, PIDConstants pidConstants) {
    sim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
    controller = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    sim.update(0.02);

    inputs.masterAppliedVolts = appliedVoltage;
    inputs.masterCurrentAmps = sim.getCurrentDrawAmps();
    inputs.masterVelocityRadPerSec = sim.getAngularVelocityRPM() / 60 * kInchesPerRotation;
    inputs.masterPositionRad = sim.getAngularPositionRotations() * kInchesPerRotation;
  }

  @Override
  public void setClimberVelocity(double voltage) {
    appliedVoltage = voltage;
    sim.setInputVoltage(voltage);
  }
}
