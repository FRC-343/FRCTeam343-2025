package frc.robot.bobot_state;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.Vision.TargetWithSource;
import frc.robot.util.VirtualSubsystem;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Class full of static variables and methods that store robot state we'd need across mulitple
 * subsystems. It's called `BobotState` as to not conflict with WPILib's {@link
 * edu.wpi.first.wpilibj.RobotState}
 */
public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  // private static final SpeakerInterpolator speakerInterpolator = new SpeakerInterpolator();
  // private static final FloorInterpolator floorInterpolator = new FloorInterpolator();

  // private static final Map<String, TargetInterpolator> targetInterpolators =
  //     Map.of(
  //         "Speaker", speakerInterpolator,
  //         "Floor", floorInterpolator);

  private static Pose2d robotPose = new Pose2d();

  /** {@link #robotPose} predicted ahead via a pose expontential of our current velocity */
  private static Pose2d predictedPose = new Pose2d();

  private static Set<TargetWithSource> visibleAprilTags = new HashSet<>();

  private static Optional<PhotonTrackedTarget> closestObject = Optional.empty();

  private static boolean isElevatorUp = false;

  private static boolean isClimberEngaged = false;

  private static AimingMode aimingMode = AimingMode.NONE;

  public static void updateRobotPose(Pose2d pose) {
    robotPose = pose;
  }

  public static Pose2d getRobotPose() {
    return robotPose;
  }

  public static void updatePredictedPose(Pose2d pose) {
    predictedPose = pose;
  }

  public static Pose2d getPredictedPose() {
    return predictedPose;
  }

  public static void updateVisibleAprilTags(Set<TargetWithSource> trackedAprilTags) {
    visibleAprilTags = trackedAprilTags;
  }

  public static Set<TargetWithSource> getVisibleAprilTags() {
    return visibleAprilTags;
  }

  public static void updateClosestObject(Optional<PhotonTrackedTarget> target) {
    closestObject = target;
  }

  public static Optional<PhotonTrackedTarget> getClosestObject() {
    return closestObject;
  }

  // public static InterpolatedCalculation getSpeakerCalculation() {
  //   return speakerInterpolator.getCalculation();
  // }

  // public static InterpolatedCalculation getFloorCalculation() {
  //   return floorInterpolator.getCalculation();
  // }

  public static void updateAimingMode(AimingMode newAimingMode) {
    aimingMode = newAimingMode;
  }

  public static AimingMode getAimingMode() {
    return aimingMode;
  }

  /**
   * Vision Assisted Rotation Correction (VARC) for PathPlanner rotation overrides
   *
   * <p>https://pathplanner.dev/pplib-override-target-rotation.html
   */
  public static Optional<Rotation2d> VARC() {
    return switch (BobotState.getAimingMode()) {
      case NONE -> Optional.empty();
      default -> Optional.empty();
    };
  }

  @Override
  public void periodic() {
    {
      String calcLogRoot = logRoot + "RobotOdometry/";
      Logger.recordOutput(calcLogRoot + "Estimated", robotPose);
      Logger.recordOutput(calcLogRoot + "Predicted", predictedPose);
    }

    // targetInterpolators.forEach(
    //     (String name, TargetInterpolator interpolator) -> {
    //       interpolator.update(robotPose);
    //       InterpolatedCalculation calculation = interpolator.getCalculation();

    //       double distanceFromSpeaker = interpolator.getDistanceFromTarget();

    //       String calcLogRoot = logRoot + "Interpolators/" + name + "/";
    //       Logger.recordOutput(calcLogRoot + "DistanceMeters", distanceFromSpeaker);
    //       Logger.recordOutput(
    //           calcLogRoot + "DistanceFeet", Units.metersToFeet(distanceFromSpeaker));
    //       Logger.recordOutput(calcLogRoot + "AngleDegrees", calculation.angleDegrees());
    //       Logger.recordOutput(calcLogRoot + "LeftSpeedRotPerSec",
    // calculation.leftSpeedRotPerSec());
    //       Logger.recordOutput(
    //           calcLogRoot + "RightSpeedRotPerSec", calculation.rightSpeedRotPerSec());
    //     });

    {
      String calcLogRoot = logRoot + "VisionCalculations/";
      Logger.recordOutput(
          calcLogRoot + "AprilTags",
          visibleAprilTags.stream()
              .map((TargetWithSource source) -> source.target().getFiducialId())
              .collect(Collectors.toList())
              .stream()
              .mapToInt(Integer::intValue)
              .distinct()
              .sorted()
              .toArray());
    }

    {
      Logger.recordOutput(logRoot + "AimingMode", BobotState.getAimingMode());
    }
  }

  public static boolean isElevatorUp() {
    return BobotState.isElevatorUp;
  }

  public static boolean isElevatorDown() {
    return !BobotState.isElevatorUp;
  }

  public static void setElevatorUp(boolean isElevatorUp) {
    BobotState.isElevatorUp = isElevatorUp;
  }

  public static boolean isClimberEngaged() {
    return BobotState.isClimberEngaged;
  }

  public static boolean isClimberDisengaged(){
    return !BobotState.isClimberEngaged;
  }

  public static void setClibmerEngaged(boolean isClimberEngaged){
    BobotState.isClimberEngaged = isClimberEngaged;
  }

  @Override
  public void simulationPeriodic() {}
}
