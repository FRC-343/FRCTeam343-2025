package frc.robot.field;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.field.FieldConstants.AprilTagStruct;
import frc.robot.util.PoseUtils;

public class HPSFace {
  public final AprilTagStruct tag;
  public final HPSLocation hps;

  public HPSFace(AprilTagStruct tag) {
    this.tag = tag;
    this.hps = new HPSLocation(tag);
  }

  public double getPerpendicularError(Pose2d robotPose) {
    return PoseUtils.getPerpendicularError(robotPose, tag.pose().toPose2d());
  }
}
