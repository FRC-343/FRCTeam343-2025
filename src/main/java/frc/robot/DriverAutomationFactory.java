package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PositionWithCoralStation;
import frc.robot.commands.PositionWithReef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.OffsetTags;
import frc.robot.util.CommandCustomController;
import frc.robot.util.MetalUtils;
import java.util.Set;

public class DriverAutomationFactory {
  private final CommandCustomController driverController;
  // private final CommandCustomController operatorController;

  private final Drive drive;

  public DriverAutomationFactory(
      CommandCustomController driverController,
      CommandCustomController operatorController,
      Drive drive) {
    this.driverController = driverController;
    // this.operatorController = operatorController;
    this.drive = drive;
  }

  public Command LeftCoralPath() {
    return MetalUtils.getCoralTag().getDeferredCommand();
  }

  public Command LeftCoralAssist() {
    return Commands.defer(
        () ->
            new PositionWithCoralStation(
                () -> -driverController.getLeftX(), drive, MetalUtils.getCoralTag()),
        Set.of(drive));
  }

  public Command ReefOnePath() {
    return OffsetTags.REEF_ONE.getDeferredCommand();
  }

  public Command ReefOneAssist() {
    return Commands.defer(
        () -> new PositionWithReef(() -> -driverController.getLeftX(), drive, OffsetTags.REEF_ONE),
        Set.of(drive));
  }
}
