package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ReefPublisher {

  private final NetworkTable smartDashboard;

  public ReefPublisher() {
    // Initialize NetworkTables and reference the SmartDashboard table
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    this.smartDashboard = inst.getTable("SmartDashboard");
  }

  public void publishQuickReefOne() {
    String quickReef = getQuickReefOne();
    NetworkTableEntry reefEntry = smartDashboard.getEntry("QuickReefOne");
    reefEntry.setString(quickReef);
  }

  public static String getQuickReefOne() {
    int station = MetalUtils.getStation(); // Replace with your actual implementation
    if (station == 1) {
      return "REEF_FIVE";
    }
    if (station == 2) {
      return "REEF_FOUR";
    }
    if (station == 3) {
      return "REEF_TWO";
    } else {
      return "REEF_FIVE";
    }
  }

  public static void main(String[] args) {
    ReefPublisher reefPublisher = new ReefPublisher();

    // Simulate periodically publishing the value to the dashboard
    while (true) {
      reefPublisher.publishQuickReefOne();

      // Add a delay to simulate periodic updates
      try {
        Thread.sleep(100); // 100ms delay
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }
}
