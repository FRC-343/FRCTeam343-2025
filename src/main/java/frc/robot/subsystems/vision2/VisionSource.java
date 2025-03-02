package frc.robot.subsystems.vision2;

import edu.wpi.first.math.geometry.Transform3d;

public record VisionSource(String name, Transform3d robotToCamera) {}
