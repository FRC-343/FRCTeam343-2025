package frc.robot.subsystems.vision2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.List;
import java.util.Optional;
import org.photonvision.simulation.VisionSystemSim;

public class VisionConstants {
  public static final record AprilTagCameraConfig(VisionSource source, SimCameraConfig simConfig) {}

  public static enum PoseEstimationMethod {
    MULTI_TAG,
    SINGLE_TAG
  }

  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final Optional<VisionSystemSim> aprilTagSim =
      Constants.currentMode == Mode.SIM
          ? Optional.of(new VisionSystemSim("AprilTagSim"))
          : Optional.empty();

  private static final List<AprilTagCameraConfig> riptideConfigs =
      List.of(
          // FLO
          new AprilTagCameraConfig(
              new VisionSource(
                  "BLeft",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-4.059), // forward+
                          Units.inchesToMeters(13.242), // left+
                          Units.inchesToMeters(27.897)), // up+
                      new Rotation3d(0, Units.degreesToRadians(5), Units.degreesToRadians(30)))),
              SimCameraConfig.ARDUCAM_OV9281_70),
          // FLI
          new AprilTagCameraConfig(
              new VisionSource(
                  "FLeft",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(10.806), // forward+
                          Units.inchesToMeters(11.934), // left+
                          Units.inchesToMeters(7.848)), // up+
                      new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-30)))),
              SimCameraConfig.ARDUCAM_OV9281_70),
          // FRI

          // FRO
          new AprilTagCameraConfig(
              new VisionSource(
                  "FRight",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(10.806), // forward+
                          Units.inchesToMeters(-11.934), // left+
                          Units.inchesToMeters(7.848)), // up+
                      new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(30)))),
              SimCameraConfig.ARDUCAM_OV9281_70),
          new AprilTagCameraConfig(
              new VisionSource(
                  "BRight",
                  new Transform3d(
                      new Translation3d(
                          Units.inchesToMeters(-4.059), // forward+
                          Units.inchesToMeters(-13.242), // left+
                          Units.inchesToMeters(27.897)), // up+
                      new Rotation3d(0, Units.degreesToRadians(5), Units.degreesToRadians(-30)))),
              SimCameraConfig.ARDUCAM_OV9281_70));

  public static final List<AprilTagCameraConfig> aprilTagCamerasConfigs = riptideConfigs;

  public static final double ambiguityCutoff = 0.06;
  public static final double singleTagPoseCutoffMeters = 4;

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
