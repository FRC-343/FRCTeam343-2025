package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.OffsetTags;
import frc.robot.subsystems.vision.VisionConstants;
// import frc.robot.Constants.PathPlannerConstants;

public class MetalUtils {
  /** Simpler way to get current alliance, or return our predetermined "DEFAULT" alliance. */
  public static Alliance getAlliance() {
    return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : null;
  }

  public static boolean isBlueAlliance() {
    return MetalUtils.getAlliance() == Alliance.Blue;
  }

  public static boolean isRedAlliance() {
    return MetalUtils.getAlliance() == Alliance.Red;
  }

  public static double getFlipped() {
    return MetalUtils.isRedAlliance() ? -1 : 1;
  }


  
  public static int getStation() {
    if (DriverStation.getAlliance().isPresent() == true) {
      return DriverStation.getLocation().getAsInt();
    } else {
      return 4;
    }
  }

  public static double percentWithSoftStops(
      double percentDecimal, double position, double min, double max) {
    boolean canMoveUp = (percentDecimal > 0.0 && position < max);
    boolean canMoveDown = (percentDecimal < 0.0 && position > min);
    return (canMoveUp || canMoveDown) ? percentDecimal : 0.0;
  }

  public static int getREEFTag() {
    return MetalUtils.isBlueAlliance()
        ? VisionConstants.BLUE_REEF_ONE
        : VisionConstants.RED_REEF_ONE;
  }

  public static OffsetTags getCoralTag() {
    if (MetalUtils.getStation() == 1) {
      return OffsetTags.CORAL_ONE;
    }
    if (MetalUtils.getStation() == 2) {
      return OffsetTags.CORAL_ONE;
    }
    if (MetalUtils.getStation() == 3) {
      return OffsetTags.CORAL_TWO;
    } else {
      return OffsetTags.CORAL_ONE;
    }
  }

  public static OffsetTags getQuickReefOne() {
    if (MetalUtils.getStation() == 1) {
      return OffsetTags.REEF_FIVE;
    }
    if (MetalUtils.getStation() == 2) {
      return OffsetTags.REEF_FOUR;
    }
    if (MetalUtils.getStation() == 3) {
      return OffsetTags.REEF_TWO;
    } else {
      return OffsetTags.REEF_FIVE;
    }
  }

  public static String getQuickReefOneTAGv() {
    if (MetalUtils.getStation() == 1) {
      return "5";
    }
    if (MetalUtils.getStation() == 2) {
      return "4";
    }
    if (MetalUtils.getStation() == 3) {
      return "2";
    } else {
      return "5";
    }
  }

  public static OffsetTags getQuickReefTwo() {
    if (MetalUtils.getStation() == 1) {
      return OffsetTags.REEF_FOUR;
    }
    if (MetalUtils.getStation() == 2) {
      return OffsetTags.REEF_THREE;
    }
    if (MetalUtils.getStation() == 3) {
      return OffsetTags.REEF_ONE;
    } else {
      return OffsetTags.REEF_FOUR;
    }
  }

  public static String getQuickReefTwoTAGv() {
    if (MetalUtils.getStation() == 1) {
      return "4";
    }
    if (MetalUtils.getStation() == 2) {
      return "3";
    }
    if (MetalUtils.getStation() == 3) {
      return "1";
    } else {
      return "4";
    }
  }

  public static OffsetTags getQuickReefThree() {
    if (MetalUtils.getStation() == 1) {
      return OffsetTags.REEF_SIX;
    }
    if (MetalUtils.getStation() == 2) {
      return OffsetTags.REEF_FIVE;
    }
    if (MetalUtils.getStation() == 3) {
      return OffsetTags.REEF_THREE;
    } else {
      return OffsetTags.REEF_SIX;
    }
  }

  public static String getQuickReefThreeTAGv() {
    if (MetalUtils.getStation() == 1) {
      return "6";
    }
    if (MetalUtils.getStation() == 2) {
      return "5";
    }
    if (MetalUtils.getStation() == 3) {
      return "3";
    } else {
      return "6";
    }
  }
}
