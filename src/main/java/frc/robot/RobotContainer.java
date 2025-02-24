package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Auto.Test;
import frc.robot.bobot_state.BobotState;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.CommandCustomController;
import frc.robot.util.Constant;
import frc.robot.util.MetalUtils;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  @SuppressWarnings("unused")
  private final BobotState m_BobotState;

  private final Elevator elevator;

  private final Climber climber;

  private final Intake intake;

  // Controller
  private final CommandCustomController controller = new CommandCustomController(0);

  private final CommandCustomController controller2 = new CommandCustomController(1);

  private final DriverAutomationFactory m_Automation;
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision("FLeft", VisionConstants.robotToCamera0),
                new VisionIOPhotonVision("FRight", VisionConstants.robotToCamera1),
                new VisionIOPhotonVision("BLeft", VisionConstants.robotToCamera2),
                new VisionIOPhotonVision("BRight", VisionConstants.robotToCamera3));
        m_BobotState = new BobotState();
        m_Automation = new DriverAutomationFactory(controller, controller2, drive);
        elevator = new Elevator();
        intake = new Intake();
        climber = new Climber();

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name, VisionConstants.robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera3Name, VisionConstants.robotToCamera3, drive::getPose));

        m_BobotState = new BobotState();
        m_Automation = new DriverAutomationFactory(controller, controller2, drive);

        elevator = new Elevator();
        intake = new Intake();
        climber = new Climber();
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        m_BobotState = new BobotState();
        m_Automation = new DriverAutomationFactory(controller, controller2, drive);

        elevator = new Elevator();
        intake = new Intake();
        climber = new Climber();
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Test Auto Pathing", new Test(elevator, intake));

    // SmartDashboard.putData(MetalUtils.getQuickReefOne());

    // Configure the button bindings
    configureButtonBindings();
    configureNamedCommands();

    SmartDashboard.putString("QuickReefOne", MetalUtils.getQuickReefOneTAGv());
    SmartDashboard.putString("QuickReefTwo", MetalUtils.getQuickReefTwoTAGv());
    SmartDashboard.putString("QuickReefThree", MetalUtils.getQuickReefThreeTAGv());
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("QuickReefCenter", m_Automation.quickReefOnePath());
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // intake.setDefaultCommand(getAutonomousCommand());

    // elevator.setVelocityCommand(controller2.getLeftY());
    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    controller.y().whileTrue(m_Automation.quickCoralPath());

    controller.a().whileTrue(m_Automation.quickReefOnePath());
    controller.b().whileTrue(m_Automation.quickReefTwoPath());
    controller.x().whileTrue(m_Automation.quickReefThreePath());

    controller.rightTrigger().whileTrue(m_Automation.processor());
    // Operator Controlls

    // "Intake" Controlls
    controller2
        .a()
        .and(controller2.leftBumper().negate())
        .whileTrue(intake.setPercentOutputThenStopCommand(-.5));

    controller2
        .y()
        .and(controller2.leftBumper().negate())
        .whileTrue(intake.setPercentOutputThenStopCommand(.5));

    // Elevator buttons

    controller2
        .leftTrigger()
        .and(controller2.leftBumper())
        .whileTrue(elevator.setElevatorPosition(Constant.elevatorConstants.L1AlgeaLevel));

    controller2
        .rightTrigger()
        .and(controller2.leftBumper())
        .whileTrue(elevator.setElevatorPosition(Constant.elevatorConstants.L2AlgeaLevel));

    controller2
        .a()
        .and(controller2.leftBumper())
        .whileTrue(elevator.setElevatorPosition(Constant.elevatorConstants.L3Level));

    controller2
        .x()
        .and(controller2.leftBumper())
        .whileTrue(elevator.setElevatorPosition(Constant.elevatorConstants.L4Level));

    controller2.b().and(controller2.leftBumper()).whileTrue(elevator.setElevatorPosition(6));

    controller2.y().and(controller2.leftBumper()).whileTrue(elevator.setElevatorPosition(.05));

    controller2
        .pov(0)
        .and(controller2.leftBumper())
        .whileTrue(elevator.setPercentOutputCommand(.06))
        .onFalse(elevator.setPercentOutputCommand(0.0));

    controller2
        .pov(180)
        .and(controller2.leftBumper())
        .whileTrue(elevator.setPercentOutputCommand(-.06))
        .onFalse(elevator.setPercentOutputCommand(0.0));

    // Climber Buttons

    controller2
        .pov(180)
        .and(controller2.rightBumper())
        .whileTrue(climber.setPercentOutputCommand(-.2))
        .whileFalse(climber.setPercentOutputCommand(0));

    controller2
        .pov(0)
        .and(controller2.rightBumper())
        .whileTrue(climber.setPercentOutputCommand(1))
        .whileFalse(climber.setPercentOutputCommand(0));

    controller2.b().and(controller2.rightBumper()).onTrue(climber.Disengage());
    controller2.x().and(controller2.rightBumper()).onTrue(climber.Engage());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void playMusic() {
    intake.playMusic();
  }

  public void pauseMusic() {
    intake.pauseMusic();
  }
}
