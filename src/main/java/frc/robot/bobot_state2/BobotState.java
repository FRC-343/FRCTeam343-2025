package frc.robot.bobot_state2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.bobot_state2.varc.BargeTagTracker;
import frc.robot.bobot_state2.varc.HPSTagTracker;
import frc.robot.bobot_state2.varc.ProcessorTagTracker;
import frc.robot.bobot_state2.varc.ReefTagTracker;
import frc.robot.bobot_state2.varc.TargetAngleTracker;
import frc.robot.field.FieldConstants;
import frc.robot.field.FieldUtils;
import frc.robot.subsystems.vision2.PoseObservation;
import frc.robot.util.VirtualSubsystem;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.LinkedBlockingQueue;
import org.littletonrobotics.junction.Logger;

/**
 * Class full of static variables and methods that store robot state we'd need across mulitple
 * subsystems. It's called {@link #BobotState} as to not conflict with WPILib's {@link
 * edu.wpi.first.wpilibj.RobotState}
 */
public class BobotState extends VirtualSubsystem {
  private static final String logRoot = "BobotState/";

  private static final Queue<PoseObservation> poseObservations = new LinkedBlockingQueue<>(20);

  private static Pose2d globalPose = new Pose2d();

  private static boolean ElevatorBeam;

  private static boolean climberState;

  private static boolean atWantedPerpPose;

  private static boolean atWantedRot;

  private static boolean atWantedParaPose;

  private static boolean elevatorSlowDown;

  private static ReefTagTracker reefTracker = new ReefTagTracker();
  private static HPSTagTracker hpsTracker = new HPSTagTracker();
  private static BargeTagTracker bargeTracker = new BargeTagTracker();
  private static ProcessorTagTracker processorTracker = new ProcessorTagTracker();

  private static List<TargetAngleTracker> autoAlignmentTrackers =
      List.of(
          BobotState.hpsTracker,
          BobotState.reefTracker,
          BobotState.processorTracker,
          BobotState.bargeTracker);

  public static void updateElevatorState(boolean state) {
    BobotState.elevatorSlowDown = state;
  }

  public static void updateElevatorBeam(boolean beam) {
    BobotState.ElevatorBeam = beam;
  }

  public static void updateWantedPose(boolean perpPoseWanted) {
    BobotState.atWantedPerpPose = perpPoseWanted;
  }

  public static void updateWantedParaPose(boolean paraPoseWanted) {
    BobotState.atWantedParaPose = paraPoseWanted;
  }

  public static void updateWantedRot(boolean rotWanted) {
    BobotState.atWantedRot = rotWanted;
  }

  public static void updateClimberState(boolean state) {
    BobotState.climberState = state;
  }

  public static void offerVisionObservation(PoseObservation observation) {
    BobotState.poseObservations.offer(observation);
  }

  public static Queue<PoseObservation> getVisionObservations() {
    return BobotState.poseObservations;
  }

  public static void updateGlobalPose(Pose2d pose) {
    BobotState.globalPose = pose;
  }

  public static Pose2d getGlobalPose() {
    return BobotState.globalPose;
  }

  public static boolean getClimberState() {
    return BobotState.climberState;
  }

  public static Trigger atWantedPose() {
    return new Trigger(
        () -> BobotState.atWantedParaPose && BobotState.atWantedPerpPose && BobotState.atWantedRot);
  }

  public static Trigger onTeamSide() {
    return new Trigger(
        () ->
            FieldUtils.getAlliance() == Alliance.Blue
                ? getGlobalPose().getX() < FieldConstants.fieldLength / 2.0
                : getGlobalPose().getX() > FieldConstants.fieldLength / 2.0);
  }

  public static Rotation2d getRotationToProcessor() {
    return BobotState.processorTracker.getRotationTarget();
  }

  public static Rotation2d getRotationToClosestReef() {
    return BobotState.reefTracker.getRotationTarget();
  }

  public static Rotation2d getRotationToClosestHPS() {
    return BobotState.hpsTracker.getRotationTarget();
  }

  public static Rotation2d getRotationToClosestBarge() {
    return BobotState.bargeTracker.getRotationTarget();
  }

  public static double getDistanceMetersFromClosestHPS() {
    return BobotState.hpsTracker.getDistanceMeters();
  }

  public static Trigger humanPlayerShouldThrow() {
    return new Trigger(() -> BobotState.hpsTracker.getDistanceMeters() < 0.5);
  }

  public static Trigger nearHumanPlayer() {
    return new Trigger(() -> BobotState.hpsTracker.getDistanceMeters() < 1);
  }

  public static TargetAngleTracker getClosestAlignmentTracker() {
    return autoAlignmentTrackers.stream()
        .reduce((a, b) -> a.getDistanceMeters() < b.getDistanceMeters() ? a : b)
        .get();
  }

  public static Trigger ElevatorBeam() {
    System.out.println(BobotState.ElevatorBeam);
    return new Trigger(() -> BobotState.ElevatorBeam);
  }

  public static Trigger ElevatorSlowdown() {
    return new Trigger(() -> BobotState.elevatorSlowDown);
  }

  @Override
  public void periodic() {

    Logger.recordOutput(logRoot + "Elevator Slowdown", elevatorSlowDown);

    Logger.recordOutput(logRoot + "Climber limit", climberState);

    Logger.recordOutput(logRoot + "Elevator Beam", ElevatorBeam);

    Logger.recordOutput(logRoot + "Wanted Perp Pose", atWantedPerpPose);

    Logger.recordOutput(logRoot + "Wanted Para Pose", atWantedParaPose);

    Logger.recordOutput(logRoot + "Wanted Rot", atWantedRot);

    Logger.recordOutput(logRoot + "Wanted Pose", atWantedPose());
    {
      processorTracker.update();

      String calLogRoot = logRoot + "Processor";
      Logger.recordOutput(calLogRoot + "Processor", FieldUtils.getProcessorFace().tag);
      Logger.recordOutput(
          calLogRoot + "TargetAngleDeg", processorTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calLogRoot + "TargetAngleRad", processorTracker.getRotationTarget().getRadians());
    }

    {
      reefTracker.update();

      String calcLogRoot = logRoot + "Reef/";
      Logger.recordOutput(calcLogRoot + "ClosestTag", FieldUtils.getClosestReef().tag);
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", reefTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", reefTracker.getRotationTarget().getRadians());
      Logger.recordOutput(calcLogRoot + "Left Pole", FieldUtils.getClosestReef().leftPole);
      Logger.recordOutput(calcLogRoot + "Right Pole", FieldUtils.getClosestReef().rightPole);
    }

    {
      hpsTracker.update();

      String calcLogRoot = logRoot + "HPS/";
      Logger.recordOutput(calcLogRoot + "Closest Tag", FieldUtils.getClosestHPSTag().HPS);
      Logger.recordOutput(calcLogRoot + "Distance", BobotState.hpsTracker.getDistanceMeters());
      Logger.recordOutput(calcLogRoot + "IsClose", BobotState.nearHumanPlayer());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", hpsTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", hpsTracker.getRotationTarget().getRadians());
    }

    {
      bargeTracker.update();

      String calcLogRoot = logRoot + "Barge/";
      Logger.recordOutput(
          calcLogRoot + "TargetAngleDeg", hpsTracker.getRotationTarget().getDegrees());
      Logger.recordOutput(
          calcLogRoot + "TargetAngleRad", hpsTracker.getRotationTarget().getRadians());
    }

    {
      String calcLogRoot = logRoot + "ClosestAlignment/";
      Logger.recordOutput(
          calcLogRoot + "Type", getClosestAlignmentTracker().getClass().getSimpleName());
    }
  }

  @Override
  public void simulationPeriodic() {}
}
