// package frc.robot.Auto;

// // This is unfinished

// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.Elevator.Elevator;
// import frc.robot.subsystems.Intake.Intake;
// import frc.robot.util.Constant;
// import frc.robot.util.MetalUtils;

// public class Test extends SequentialCommandGroup {
//   private static final double intakeOut = -.5;
//   private static final double intakeReverse = .5;

//   public Test(Elevator El, Intake in) {
//     Elevator m_Elevator = El;
//     Intake m_Intake = in;
//     addCommands(
//         new ParallelDeadlineGroup(
//             MetalUtils.getQuickReefThree().getDeferredCommand(),
//             new SequentialCommandGroup(
//                 new WaitCommand(1),
//                 m_Elevator.setElevatorPosition(Constant.elevatorConstants.L4Level))),
//         m_Intake.runForTime(intakeOut, 1),
//         // Grab New Coral
//         new ParallelDeadlineGroup(
//             MetalUtils.getCoralTag().getDeferredCommand(),
//             new WaitCommand(.5),
//             m_Elevator.setElevatorPosition(.1)),
//         m_Intake.setPercentOutputBeambreakCommand(.5, m_Elevator.beambreakIsObstructed()));
//     // Grab New Coral End

//   }
// }
