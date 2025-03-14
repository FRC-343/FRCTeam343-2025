package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bobot_state2.BobotState;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPoseCommand extends Command {
  private final PIDController perpendicularController =
      DriveCommandConstants.makeTranslationController();
  private final PIDController parallelController =
      DriveCommandConstants.makeTranslationController();
  private final ProfiledPIDController angleController = DriveCommandConstants.makeAngleController();

  private final Drive drive;
  private final Supplier<Pose2d> targetPoseSupplier;

  public DriveToPoseCommand(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(drive);

    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
  }

  @Override
  public void initialize() {
    BobotState.atWantedPose();
    BobotState.updateWantedParaPose(parallelController.atSetpoint());
    BobotState.updateWantedPose(perpendicularController.atSetpoint());
    BobotState.updateWantedRot(angleController.atSetpoint());
  }

  @Override
  public void execute() {
    Logger.recordOutput(
        "Commands/" + getName() + "/Accessed", BobotState.atWantedPose().getAsBoolean());

    BobotState.updateWantedParaPose(parallelController.atSetpoint());
    BobotState.updateWantedPose(perpendicularController.atSetpoint());
    BobotState.updateWantedRot(angleController.atSetpoint());

    Pose2d robotPose = drive.getPose();
    Pose2d targetPose = targetPoseSupplier.get();
    Logger.recordOutput("Commands/" + getName() + "/TargetPose", targetPose);

    Transform2d error = robotPose.minus(targetPose);
    Logger.recordOutput("Commands/" + getName() + "/Error", error);

    double perpendicularSpeed = perpendicularController.calculate(error.getX(), 0);
    perpendicularSpeed = !perpendicularController.atSetpoint() ? perpendicularSpeed : 0;

    double parallelSpeed = parallelController.calculate(error.getY(), 0);
    parallelSpeed = !parallelController.atSetpoint() ? parallelSpeed * 1.5 : 0;

    double angularSpeed = angleController.calculate(error.getRotation().getRadians(), 0);
    angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;

    ChassisSpeeds speeds = new ChassisSpeeds(perpendicularSpeed, parallelSpeed * 2, angularSpeed);

    drive.runVelocity(speeds);
  }

  @Override
  public boolean isFinished() {

    return perpendicularController.atSetpoint();
  }

  public Trigger atSetpoint() {
    return new Trigger(
        () ->
            perpendicularController.atSetpoint()
                && parallelController.atSetpoint()
                && angleController.atSetpoint());
  }
}
