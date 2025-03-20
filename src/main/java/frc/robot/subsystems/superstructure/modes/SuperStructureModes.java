package frc.robot.subsystems.superstructure.modes;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.superstructure.constants.PivotConstants;

public enum SuperStructureModes {
  TUCKED(0.9 / 2.0, PivotConstants.intialPosition),
  TUCKED_L4(SuperStructureModes.TUCKED.elevatorHeightInches, Rotation2d.fromDegrees(90)),
  L2Coral(19.0 / 2.0, SuperStructureModes.TUCKED.coralPos),
  L3Coral(36.0 / 2.0, SuperStructureModes.TUCKED.coralPos),
  L4Coral(49.5 / 2.0, Rotation2d.fromDegrees(90)),
  FLOOR_ALGAE(1.0 / 2.0, Rotation2d.fromDegrees(-40.0)),
  L2Algae(20.0 / 2.0, Rotation2d.fromDegrees(-10.0)),
  L3Algae(37.0 / 2.0, Rotation2d.fromDegrees(-10.0)),
  Barge(50.0 / 2.0, Rotation2d.fromDegrees(90));

  public final double elevatorHeightInches;
  public final Rotation2d coralPos;

  private SuperStructureModes(double elevatorHeightIn, Rotation2d coralPos) {
    this.elevatorHeightInches = elevatorHeightIn;
    this.coralPos = coralPos;
  }
}
