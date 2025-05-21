// Copyright (c) 2025 FRC 6907, The G.O.A.T
package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class VisionConstants {
  public static AprilTagFieldLayout aprilTagLayout = Constants.kFieldTagLayout;

  // ================== Constants ========================
  public static String camera0Name = "effectorLeft";
  public static String camera1Name = "effectorRight";
  public static String camera2Name = "effectorCenter";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(0.32, 0.29, 0.25, new Rotation3d(0.0, -5 * (Math.PI / 180), -0.43633));
  public static Transform3d robotToCamera1 =
      new Transform3d(0.32, -0.29, 0.25, new Rotation3d(0.0, -5 * (Math.PI / 180), 0.43633));
  public static Transform3d robotToCamera2 =
      new Transform3d(0.28, 0.0, 0.17, new Rotation3d(0.0, -30 * (Math.PI / 180), 0.0)); // 50

  // Filtering constants
  public static final double ACCEPTABLE_AMBIGUITY_THRESHOLD = 0.1;
  public static final double ACCEPTABLE_YAW_THRESHOLD = 35.0; // degrees
  public static final double ACCEPTABLE_PITCH_THRESHOLD = 25.0; // degrees
  public static final double Z_MARGIN = 0.15; // reject if abs(Z) > this
  public static final double[] MIN_AREA = new double[] {0.5, 0.5, 0.1};

  // multi-tag delayed usage
  public static final double MULTI_TAG_DELAY = 0.1; // seconds

  // velocity filtering
  public static final double STATIC_DELTAV_BOUND = 2.0; // m/s
  public static final double MOVING_DELTAV_BOUND = 3.0; // m/s

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0, // Camera 1
        1.0 // Camera 2
      };
}
