package frc.robot.subsystems.vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import frc.util.GarageUtils;
import frc.robot.util.MetalUtils;

public enum OffsetTags {
  CORAL_ONE(VisionConstants.RED_CORAL_ONE, VisionConstants.BLUE_CORAL_ONE, .5),
  CORAL_TWO(VisionConstants.RED_CORAL_TWO, VisionConstants.BLUE_CORAL_TWO, .5),
  REEF_ONE(
      VisionConstants.RED_REEF_ONE,
      VisionConstants.BLUE_REEF_ONE,
      .5,
      new Rotation3d(0, 0, Math.PI)),
  REEF_TWO(
      VisionConstants.RED_REEF_TWO,
      VisionConstants.BLUE_REEF_TWO,
      .3,
      new Rotation3d(0, 0, Math.PI)),
  REEF_THREE(
      VisionConstants.RED_REEF_THREE,
      VisionConstants.BLUE_REEF_THREE,
      .5,
      new Rotation3d(0, 0, Math.PI)),
  REEF_FOUR(
      VisionConstants.RED_REEF_FOUR,
      VisionConstants.BLUE_REEF_FOUR,
      .5,
      new Rotation3d(0, 0, Math.PI)),
  REEF_FIVE(
      VisionConstants.RED_REEF_FIVE,
      VisionConstants.BLUE_REEF_FIVE,
      .5,
      new Rotation3d(0, 0, Math.PI)),
  REEF_SIX(
      VisionConstants.RED_REEF_SIX,
      VisionConstants.BLUE_REEF_SIX,
      .5,
      new Rotation3d(0, 0, Math.PI)),
  PROCESSOR(
      VisionConstants.RED_PROCESSOR,
      VisionConstants.BLUE_PROCESSOR,
      .5,
      new Rotation3d(0, 0, Math.PI)),
  OTHER_PROCESSOR(
      VisionConstants.BLUE_PROCESSOR,
      VisionConstants.RED_PROCESSOR,
      .5,
      new Rotation3d(0, 0, Math.PI));

  private final int redId;
  private final int blueId;
  private final double poseOffsetMeters;
  private final Rotation3d extraRotation;

  private OffsetTags(int red, int blue, double poseOffsetMeters) {
    this(red, blue, poseOffsetMeters, new Rotation3d());
  }

  private OffsetTags(int red, int blue, double poseOffsetMeters, Rotation3d extraRotation) {
    this.redId = red;
    this.blueId = blue;
    this.poseOffsetMeters = poseOffsetMeters;
    this.extraRotation = extraRotation;
  }

  public Pose3d getOffsetPoseFrom(Pose3d pose) {
    return new Pose3d(
        pose.getTranslation()
            .plus(
                new Translation3d(
                    poseOffsetMeters * Math.cos(pose.getRotation().getZ()),
                    poseOffsetMeters * Math.sin(pose.getRotation().getZ()),
                    0)),
        pose.getRotation().plus(extraRotation));
  }

  public Pose3d getOffsetPose() {
    return MetalUtils.isRedAlliance() ? getRedOffsetPose() : getBlueOffsetPose();
  }

  public Pose3d getRedOffsetPose() {
    Pose3d pose = VisionConstants.aprilTagLayout.getTagPose(redId).get();
    return getOffsetPoseFrom(pose);
  }

  public Pose3d getBlueOffsetPose() {
    Pose3d pose = VisionConstants.aprilTagLayout.getTagPose(blueId).get();
    return getOffsetPoseFrom(pose);
  }

  public Command getDeferredCommand() {
    return Commands.deferredProxy(
        () ->
            AutoBuilder.pathfindToPose(
                getOffsetPose().toPose2d(),
                new PathConstraints(10, 6, 4 * Math.PI, 5 * Math.PI),
                0.0));
  }

  public Pose3d getPose() {
    return VisionConstants.aprilTagLayout.getTagPose(getId()).get();
  }

  public Pose3d getRedPose() {
    return VisionConstants.aprilTagLayout.getTagPose(getRedId()).get();
  }

  public Pose3d getBluePose() {
    return VisionConstants.aprilTagLayout.getTagPose(getBlueId()).get();
  }

  public int getId() {
    return MetalUtils.isRedAlliance() ? redId : blueId;
  }

  public int getRedId() {
    return redId;
  }

  public int getBlueId() {
    return blueId;
  }

  /**
   * Using the robot's known pose, find the distance of how far away the robot is from the {@link
   * #OffsetTags}.
   *
   * @param pose - Current Robot Pose
   * @return Distance from robot to target (meters)
   */
  public double getFieldDistanceFrom(Pose2d pose) {
    Translation2d poseTranslation = pose.getTranslation();
    Translation2d targetTranslation = getPose().toPose2d().getTranslation();
    double distanceToTarget = poseTranslation.getDistance(targetTranslation);

    return distanceToTarget;
  }

  // /**
  //  * Using the robot's known pose, find the distance of how far away the robot is from the {@link
  //  * #OffsetTags}.
  //  *
  //  * @param pose - Current Robot Pose
  //  * @return Distance from robot to target (meters)
  //  */
  // public Optional<Double> getVisionDistanceFrom(Pose2d pose) {
  //   Optional<Double> distance = Optional.empty();

  //   Optional<TargetWithSource> targetWithSource =
  //       AprilTagAlgorithms.filterTags(BobotState.getVisibleAprilTags().stream(), getId())
  //           .reduce(
  //               (targetWithSourceA, targetWithSourceB) ->
  //                   targetWithSourceA.target().getPoseAmbiguity()
  //                           <= targetWithSourceB.target().getPoseAmbiguity()
  //                       ? targetWithSourceA
  //                       : targetWithSourceB);

  //   if (targetWithSource.isPresent()) {
  //     Translation2d poseTranslation = pose.getTranslation();

  //     Pose3d tagPose = targetWithSource.get().getTargetPoseFrom(new Pose3d(pose));
  //     Pose3d targetPose = getOffsetPoseFrom(tagPose);
  //     Translation2d targetTranslation = targetPose.toPose2d().getTranslation();
  //     distance = Optional.of(poseTranslation.getDistance(targetTranslation));
  //   }

  //   return distance;
  // }

  /**
   * Using the robot's known pose, find the distance of how far away the robot is from the {@link
   * #OffsetTags}.
   *
   * @param pose - Current Robot Pose
   * @return Distance from robot to target (meters)
   */
  // public double getDistanceFrom(Pose2d pose) {
  //   return getVisionDistanceFrom(pose).orElse(getFieldDistanceFrom(pose));
  // }
}
