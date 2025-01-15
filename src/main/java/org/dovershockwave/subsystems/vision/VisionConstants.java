package org.dovershockwave.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static final AprilTagFieldLayout APRIL_TAG_FIELD = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static final String REEF_CAMERA = "ReefCamera";
  public static final String HUMAN_PLAYER_STATION_CAMERA = "HumanPlayerStationCamera";

  public static final Transform3d ROBOT_TO_REEF_CAMERA = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
  public static final Transform3d ROBOT_TO_HUMAN_PLAYER_CAMERA = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));

  public static final double MAX_AMBIGUITY = 0.3;
  public static final double MAX_Z_ERROR = 0.75;

  /**
   * Standard deviation baselines, for 1 meter distance and 1 tag (Adjusted automatically based on distance and # of tags)
   */
  public static final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
  public static final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static final double[] CAMERA_STD_DEV_FACTORS =
      new double[] {
        1.0, // Reef Camera
        1.0 // Human Player Station Camera
      };

  // Multipliers to apply for MegaTag 2 observations
  public static final double LINEAR_STD_DEV_MEGATAG_2_FACTOR = 0.5; // More stable than full 3D solve
  public static final double ANGULAR_STD_DEV_MEGATAG_2_FACTOR = Double.POSITIVE_INFINITY; // No rotation data available
}