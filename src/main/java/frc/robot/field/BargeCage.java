package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.util.PoseUtils;

public class BargeCage {
  public final AprilTagStruct tag;
  public final CageCenter leftCage;
  public final CageCenter centerCage;
  public final CageCenter rigthCage;

  public BargeCage(AprilTagStruct tag) {
    this.tag = tag;
    this.centerCage = new CageCenter(tag, FieldConstants.tagToCageCenter);
    this.leftCage = new CageCenter(tag, FieldConstants.tagToCageLeft);
    this.rigthCage = new CageCenter(tag, FieldConstants.tagToCageRight);
  }

  public double getPerpendicularError(Pose2d robotPose) {
    return PoseUtils.getPerpendicularError(robotPose, tag.pose().toPose2d());
  }
}
