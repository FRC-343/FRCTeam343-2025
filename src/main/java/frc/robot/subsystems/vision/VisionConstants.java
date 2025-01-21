package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "FcamLeft";
  public static String camera1Name = "FcamRight";
  public static String camera2Name = "BcamLeft";
  public static String camera3Name = "BcamRight";

  // Robot to camera transforms
  // This is Camera centered IE instead of XYZ its YXZ for the Transform
  public static Transform3d robotToCamera0 =
      new Transform3d( // Front left
          Units.inchesToMeters(10.713),
          Units.inchesToMeters(11.466),
          Units.inchesToMeters(8.055),
          new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(-45)));
  public static Transform3d robotToCamera1 =
      new Transform3d( // Front Right
          Units.inchesToMeters(10.713),
          Units.inchesToMeters(-11.466),
          Units.inchesToMeters(8.055),
          new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(45)));
  public static Transform3d robotToCamera2 =
      new Transform3d( // Back left
          Units.inchesToMeters(-10.713),
          Units.inchesToMeters(11.466),
          Units.inchesToMeters(8.055),
          new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(180 - 45)));
  public static Transform3d robotToCamera3 =
      new Transform3d( // Back Right
          Units.inchesToMeters(-10.713),
          Units.inchesToMeters(-11.466),
          Units.inchesToMeters(8.055),
          new Rotation3d(0.0, Units.degreesToRadians(-30), Units.degreesToRadians(180 + 45)));
  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available

  /*
   *
   *
   *  APRIL TAG ID ASSINGMENT
   *
   *
   */

  // Coral Sation IDs
  // RED
  public static final Integer RED_CORAL_ONE = 1;
  public static final Integer RED_CORAL_TWO = 2;

  public static final Set<Integer> RED_CORAL_TAGS = Set.of(RED_CORAL_ONE, RED_CORAL_TWO);
  // Blue
  public static final Integer BLUE_CORAL_ONE = 13;
  public static final Integer BLUE_CORAL_TWO = 12;

  public static final Set<Integer> BLUE_CORAL_TAGS = Set.of(BLUE_CORAL_ONE, BLUE_CORAL_TWO);

  // REEF IDs These go in a clock wise rotation when looking at it from the respective drive station
  // RED
  public static final Integer RED_REEF_ONE = 10;
  public static final Integer RED_REEF_TWO = 9;
  public static final Integer RED_REEF_THREE = 8;
  public static final Integer RED_REEF_FOUR = 7;
  public static final Integer RED_REEF_FIVE = 6;
  public static final Integer RED_REEF_SIX = 11;

  public static final Set<Integer> RED_REEF_TAGS =
      Set.of(
          RED_REEF_ONE, RED_REEF_TWO, RED_REEF_THREE, RED_REEF_FOUR, RED_REEF_FIVE, RED_REEF_SIX);
  // BLUE
  public static final Integer BLUE_REEF_ONE = 21;
  public static final Integer BLUE_REEF_TWO = 22;
  public static final Integer BLUE_REEF_THREE = 17;
  public static final Integer BLUE_REEF_FOUR = 18;
  public static final Integer BLUE_REEF_FIVE = 19;
  public static final Integer BLUE_REEF_SIX = 20;

  public static final Set<Integer> BLUE_REEF_TAGS =
      Set.of(
          BLUE_REEF_ONE,
          BLUE_REEF_TWO,
          BLUE_REEF_THREE,
          BLUE_REEF_FOUR,
          BLUE_REEF_FIVE,
          BLUE_REEF_SIX);
  // Processor IDs
  // RED
  public static final Integer RED_PROCESSOR = 3;
  // Blue
  public static final Integer BLUE_PROCESSOR = 16;
  // Barge IDs
  // RED
  public static final Integer RED_BARGE_ONE = 5;
  public static final Integer RED_BARGE_TWO = 15;

  public static final Set<Integer> RED_BARGE_TAGS = Set.of(RED_BARGE_ONE, RED_BARGE_TWO);
  // BLUE
  public static final Integer BLUE_BARGE_ONE = 14;
  public static final Integer BLUE_BARGE_TWO = 4;

  public static final Set<Integer> BLUE_BARGE_TAGS = Set.of(BLUE_BARGE_ONE, BLUE_BARGE_TWO);

  // Field IDs

  // RED TAG FIDS
  public static final Set<Integer> RED_TAG_FIDS =
      Stream.of(RED_BARGE_TAGS, RED_CORAL_TAGS, RED_REEF_TAGS, Set.of(RED_PROCESSOR))
          .flatMap(Set::stream)
          .collect(Collectors.toSet());
  // BLUE TAG FIDS
  public static final Set<Integer> BLUE_TAG_FIDS =
      Stream.of(BLUE_BARGE_TAGS, BLUE_CORAL_TAGS, BLUE_REEF_TAGS, Set.of(BLUE_PROCESSOR))
          .flatMap(Set::stream)
          .collect(Collectors.toSet());

  // All Tags
  public static final Set<Integer> ALL_TAGS =
      Stream.concat(RED_TAG_FIDS.stream(), BLUE_TAG_FIDS.stream()).collect(Collectors.toSet());

  // List of all possible combinations of april tags
  public static final List<Set<Integer>> POSSIBLE_FRAME_FID_COMBOS =
      List.of(RED_TAG_FIDS, BLUE_TAG_FIDS);

  /***********************************************************************
   * Represents parameters for computing unit deviation
   * based on average distance.
   *
   * The record encapsulates three parameters:
   * - distanceMultiplier: Multiplier applied to the average distance.
   * - eulerMultiplier: Multiplier applied to the exponential term.
   * - minimum: Minimum value for the computed unit deviation.
   *
   * ComputeUnitDeviation calculates unit deviation using the
   * provided average distance according to the formula:
   *
   * max(minimum, eulerMultiplier * exp(averageDistance * distanceMultiplier)).
   */
  public static record UnitDeviationParams(
      double distanceMultiplier, double eulerMultiplier, double minimum) {
    /**
     * Computes unit deviation based on the average distance param.
     *
     * @param averageDistance The average distance used in the computation.
     * @return The computed unit deviation.
     */
    private double ComputeUnitDeviation(double averageDistance) {
      return Math.max(minimum, eulerMultiplier * Math.exp(averageDistance * distanceMultiplier));
    }
  }

  /***********************************************************************
   * Represents deviation parameters for tags in X, Y, and Theta.
   *
   * This record encapsulates three sets of deviation parameters:
   * - xParams: Parameters for X dimension.
   * - yParams: Parameters for Y dimension.
   * - thetaParams: Parameters for Theta.
   *
   * The ComputeDeviation method calculates the deviation matrix based
   * on the provided average distance. The resulting matrix represents
   * deviations for X, Y, and Theta dimensions.
   *
   * The constructor TagCountDeviation(xyParams, thetaParams)
   * initializes the deviation parameters for X and Y dimensions
   * using the same set of parameters.
   */
  public static record TagCountDeviation(
      UnitDeviationParams xParams, UnitDeviationParams yParams, UnitDeviationParams thetaParams) {
    /**
     * Computes the deviation matrix based on the average distance parameter.
     *
     * @param averageDistance The average distance used in the computation.
     * @return The deviation matrix for X, Y, and Theta dimensions.
     */
    public Matrix<N3, N1> computeDeviation(double averageDistance) {
      return MatBuilder.fill(
          Nat.N3(),
          Nat.N1(),
          xParams.ComputeUnitDeviation(averageDistance),
          yParams.ComputeUnitDeviation(averageDistance),
          thetaParams.ComputeUnitDeviation(averageDistance));
    }

    /**
     * Constructor that initializes deviation parameters for X and Y dimensions using the same set
     * of parameters.
     *
     * @param xyParams Parameters for X and Y dimensions.
     * @param thetaParams Parameters for Theta dimension.
     */
    public TagCountDeviation(UnitDeviationParams xyParams, UnitDeviationParams thetaParams) {
      this(xyParams, xyParams, thetaParams);
    }
  }

  /**
   * The TAG_COUNT_DEVIATION_PARAMS list contains instances of TagCountDeviation, each representing
   * deviation parameters for tags in X, Y, and Theta dimensions based on the number of tags
   * detected. The list is organized by the count of tags: - For 1 tag: Parameters for X, Y, and
   * Theta dimensions are specified. - For 2 tags: Parameters for X and Y dimensions are specified,
   * using the same set of parameters. - For 3 or more tags: Parameters for X and Y dimensions are
   * specified, using the same set of parameters.
   *
   * <p>Each TagCountDeviation instance contains three sets of UnitDeviationParams: - xParams:
   * Parameters for X dimension. - yParams: Parameters for Y dimension. - thetaParams: Parameters
   * for Theta dimension.
   *
   * <p>The deviation matrix can be computed using the ComputeDeviation method for a given average
   * distance. The resulting matrix represents deviations for X, Y, and Theta dimensions.
   *
   * <p>Example Usage: List<TagCountDeviation> deviations = TAG_COUNT_DEVIATION_PARAMS;
   * TagCountDeviation paramsFor1Tag = deviations.get(0); Matrix<N3, N1> deviationMatrix =
   * paramsFor1Tag.ComputeDeviation(averageDistance);
   */
  public static final List<TagCountDeviation> TAG_COUNT_DEVIATION_PARAMS =
      List.of(
          // 1 tag
          new TagCountDeviation(
              new UnitDeviationParams(.25, .4, .9),
              new UnitDeviationParams(.35, .5, 1.2),
              new UnitDeviationParams(.5, .7, 1.5)),

          // 2 tags
          new TagCountDeviation(
              new UnitDeviationParams(.35, .1, .4), new UnitDeviationParams(.5, .7, 1.5)),

          // 3+ tags
          new TagCountDeviation(
              new UnitDeviationParams(.25, .07, .25), new UnitDeviationParams(.15, 1, 1.5)));

  public static final int MAX_FRAME_FIDS = 22;
  public static final double POSE_AMBIGUITY_CUTOFF = .05;
}
