package frc.robot.subsystems.vision2;

import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class AprilTagIOPhotonSim extends AprilTagIOPhoton {
  private final PhotonCameraSim cameraSim;

  public AprilTagIOPhotonSim(VisionSource source, SimCameraConfig config) {
    super(source);

    SimCameraProperties props = config.apply(new SimCameraProperties());

    cameraSim = new PhotonCameraSim(camera, props, VisionConstants.fieldLayout);

    cameraSim.enableDrawWireframe(true);
    cameraSim.setMaxSightRange(10.0);
    cameraSim.setWireframeResolution(1);

    VisionConstants.aprilTagSim.ifPresent(
        aprilTagSim -> aprilTagSim.addCamera(cameraSim, source.robotToCamera()));
  }

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    super.updateInputs(inputs);

    VisionConstants.aprilTagSim.ifPresent(
        aprilTagSim -> {
          FieldObject2d visionEstimation =
              aprilTagSim.getDebugField().getObject("VisionEstimation");

          if (inputs.validPoseObservations.length != 0) {
            visionEstimation.setPoses(inputs.validPoseObservations[0].robotPose().toPose2d());
          } else {
            visionEstimation.setPoses();
          }
        });
  }
}
