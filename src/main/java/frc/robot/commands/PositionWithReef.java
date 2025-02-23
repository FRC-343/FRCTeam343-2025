package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.bobot_state.BobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.AprilTagAlgorithms;
import frc.robot.subsystems.vision.OffsetTags;
import frc.robot.subsystems.vision.Vision.TargetWithSource;
import frc.robot.util.MetalUtils;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class PositionWithReef extends Command {
  private static double yawMeasurementOffset = Math.PI; // To aim from the back
  private final PIDController xController = new PIDController(1, 0, 0);
  private final PIDController thetaController = new PIDController(5, 0, 0.1);
  private final String logRoot;

  private final Drive drive;
  private final OffsetTags tag;
  // private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;

  private Pose3d tagPose = new Pose3d();
  private Pose3d targetPose = new Pose3d();
  private boolean hasSeenTag = false;

  public PositionWithReef(DoubleSupplier ySupplier, Drive drive, OffsetTags tag) {
    addRequirements(drive);
    setName("PositionWithReef");

    logRoot = "Commands/" + getName() + "/";

    this.ySupplier = ySupplier;
    this.tag = tag;
    this.drive = drive;

    xController.setTolerance(0.1);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    drive.runVelocity(new ChassisSpeeds());

    tagPose = tag.getPose();
    targetPose = tag.getOffsetPose();

    Logger.recordOutput(logRoot + "IsRunning", true);
  }

  @Override
  public void execute() {
    Pose3d robotPose = new Pose3d(drive.getPose());

    Set<TargetWithSource> targets = BobotState.getVisibleAprilTags();
    AprilTagAlgorithms.filterTags(targets.stream(), tag.getId())
        .reduce(
            (targetWithSourceA, targetWithSourceB) ->
                targetWithSourceA.target().getPoseAmbiguity()
                        <= targetWithSourceB.target().getPoseAmbiguity()
                    ? targetWithSourceA
                    : targetWithSourceB)
        .ifPresent(
            targetWithSource -> {
              hasSeenTag = true;
              tagPose = targetWithSource.getTargetPoseFrom(robotPose);
              targetPose = tag.getOffsetPoseFrom(tagPose);
            });

    double yawErrorRad =
        tagPose.relativeTo(robotPose).getTranslation().toTranslation2d().getAngle().getRadians();
    double rotationSpeedRad = thetaController.calculate(yawMeasurementOffset, yawErrorRad);

    double xSpeedMeters =
        MathUtil.clamp(xController.calculate(robotPose.getX(), targetPose.getX()), -5, 5);

    Logger.recordOutput(logRoot + "TargetID", tag.getId());
    Logger.recordOutput(logRoot + "TagPose", tagPose);
    Logger.recordOutput(logRoot + "TagPose2d", tagPose.toPose2d());
    Logger.recordOutput(logRoot + "TargetPose", targetPose);
    Logger.recordOutput(logRoot + "TargetPose2d", targetPose.toPose2d());
    Logger.recordOutput(logRoot + "HasSeenTarget", hasSeenTag);
    Logger.recordOutput(logRoot + "RotationSpeed", rotationSpeedRad);

    TeleopDrive.drive(
        drive,
        MetalUtils.getFlipped() * xSpeedMeters / 5,
        ySupplier.getAsDouble(),
        rotationSpeedRad / 3,
        false,
        true);
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput(logRoot + "IsRunning", false);
  }
}
