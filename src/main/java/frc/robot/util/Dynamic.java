package frc.robot.util;

import frc.robot.bobot_state2.BobotState;

public class Dynamic {
  public final class dynaPID {

    public static Double fastAngleP = 10.0;
    public static Double fastDriveP = 3.5;

    public static Double slowAngleP = 5.0;
    public static Double slowDriveP = 1.0;

    public static Double fastAngleD = .4;

    public static Double slowAngleD = .1;
  }

  public static double DynaPDriveTest() {
    if (BobotState.getNearWhat() == "HPS") {
      return Dynamic.dynaPID.slowDriveP;
    } else if (BobotState.getNearWhat() == "Reef") {
      return Dynamic.dynaPID.slowDriveP;
    } else {
      return Dynamic.dynaPID.fastDriveP;
    }
  }
}

// public class DriveCommandConstants {
//     public static ProfiledPIDController makeAngleController() {
//       ProfiledPIDController angleController =
//           new ProfiledPIDController(10.0, 0.0, .4, new TrapezoidProfile.Constraints(8.0, 20.0));

//       angleController.enableContinuousInput(-Math.PI, Math.PI);
//       angleController.setTolerance(Units.degreesToRadians(.5));

//       return angleController;
//     }

//     public static PIDController makeTranslationController() {
//       PIDController translationController = new PIDController(3.5, 0.0, 0);
//       translationController.setTolerance(Units.inchesToMeters(1));

//       return translationController;
