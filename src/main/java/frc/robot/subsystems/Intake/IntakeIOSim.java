package frc.robot.subsystems.intake;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  // private static double momentOfInertiaKgMSquared = 0.0000032998;
  private static double momentOfInertiaKgMSquared = 1.0;

  private final DCMotorSim wheelSim;

  private final PIDController controller;

  public IntakeIOSim(DCMotor motorModel, double reduction, double moi, PIDConstants pidConstants) {
    wheelSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, moi, reduction), motorModel);
    controller = new PIDController(pidConstants.kP, pidConstants.kI, pidConstants.kD);
  }

  private double velocityRotPerSecond = 0.0;

  private boolean closedLoop = false;
  private double appliedVoltage = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (closedLoop) {
      appliedVoltage =
          12.0
              * controller.calculate(wheelSim.getAngularVelocityRPM() / 60.0, velocityRotPerSecond);
    }

    wheelSim.setInputVoltage(appliedVoltage);
    wheelSim.update(0.02); // 20 ms is the standard periodic loop time

    inputs.appliedVoltage = appliedVoltage;
    inputs.appliedDutyCycle = appliedVoltage / 12.0;
    inputs.currentAmperage = wheelSim.getCurrentDrawAmps();
    inputs.velocityRotPerSecond = wheelSim.getAngularVelocityRPM() / 60.0;
  }

  @Override
  public void setVelocity(double velocityRotPerSecond) {
    closedLoop = true;
    this.velocityRotPerSecond = velocityRotPerSecond;
  }

  @Override
  public void setPercentOutput(double percentDecimal) {
    setVelocity(12 * percentDecimal);
  }

  @Override
  public void setVoltage(double voltage) {
    closedLoop = false;
    appliedVoltage = voltage;
  }
}
