package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PositionWithCoralStation;
import frc.robot.commands.PositionWithReef;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CommandCustomController;
import frc.robot.util.MetalUtils;
import java.util.Set;

public class DriverAutomationFactory {
  private final CommandCustomController driverController;
  private final CommandCustomController operatorController;

  private final Drive drive;

  public DriverAutomationFactory(
      CommandCustomController driverController,
      CommandCustomController operatorController,
      Drive drive) {
    this.driverController = driverController;
    this.operatorController = operatorController;
    this.drive = drive;
  }

  public Command quickCoralPath() {
    return MetalUtils.getCoralTag().getDeferredCommand();
  }

  public Command quickCoralAssist() {
    return Commands.defer(
        () ->
            new PositionWithCoralStation(
                () -> -driverController.getLeftX(), drive, MetalUtils.getCoralTag()),
        Set.of(drive));
  }

  public Command quickReefOnePath() {
    return MetalUtils.getQuickReefOne().getDeferredCommand();
  }

  public Command quickReefOneAssist() {
    return Commands.defer(
        () ->
            new PositionWithReef(
                () -> -driverController.getLeftX(), drive, MetalUtils.getQuickReefOne()),
        Set.of(drive));
  }

  public Command quickReefTwoPath() {
    return MetalUtils.getQuickReefTwo().getDeferredCommand();
  }

  public Command quickReefTwoAssist() {
    return Commands.defer(
        () ->
            new PositionWithReef(
                () -> -driverController.getLeftX(), drive, MetalUtils.getQuickReefTwo()),
        Set.of(drive));
  }

  public Command quickReefThreePath() {
    return MetalUtils.getQuickReefThree().getDeferredCommand();
  }

  public Command quickReefThreeAssist() {
    return Commands.defer(
        () ->
            new PositionWithReef(
                () -> -driverController.getLeftX(), drive, MetalUtils.getQuickReefThree()),
        Set.of(drive));
  }
}
