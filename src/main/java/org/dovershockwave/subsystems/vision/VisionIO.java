package org.dovershockwave.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation bestTargetObservation = new TargetObservation(Integer.MIN_VALUE, new Rotation2d(), new Rotation2d(), new Translation3d(), 0.0);
    public TargetObservation[] latestTargetObservations = new TargetObservation[0];
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /**
   * Represents the angle/distance to a simple target from a robot, not used for pose estimation
   */
  record TargetObservation(int tagId, Rotation2d tx, Rotation2d ty, Translation3d translation, double distance) {}

  /**
   * Represents a robot pose sample used for pose estimation.
   */
  record PoseObservation(double timestamp, Pose3d pose, double ambiguity, int tagCount, double averageTagDistance) {}

  default void updateInputs(VisionIOInputs inputs) {}
}