package org.dovershockwave.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.subsystems.vision.VisionIO.PoseObservationType;
import org.dovershockwave.subsystems.vision.VisionIO.TargetObservation;
import org.littletonrobotics.junction.Logger;

import java.util.LinkedList;

import static org.dovershockwave.subsystems.vision.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public VisionSubsystem(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] = new Alert("Vision camera " + i + " is disconnected.", AlertType.kWarning);
    }
  }

  public TargetObservation getLatestTargetObservation(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation;
  }

  @Override public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
    }

    // Initialize logging values
    final var allTagPoses = new LinkedList<Pose3d>();
    final var allRobotPoses = new LinkedList<Pose3d>();
    final var allRobotPosesAccepted = new LinkedList<Pose3d>();
    final var allRobotPosesRejected = new LinkedList<Pose3d>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      final var tagPoses = new LinkedList<Pose3d>();
      final var robotPoses = new LinkedList<Pose3d>();
      final var robotPosesAccepted = new LinkedList<Pose3d>();
      final var robotPosesRejected = new LinkedList<Pose3d>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = APRIL_TAG_FIELD.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        final boolean rejectPose =
                observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                        && observation.ambiguity() > MAX_AMBIGUITY) // Cannot be high ambiguity
                        || Math.abs(observation.pose().getZ())
                        > MAX_Z_ERROR // Must have realistic Z coordinate

                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > APRIL_TAG_FIELD.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > APRIL_TAG_FIELD.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) continue;

        // Calculate standard deviations
        final double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor;
        double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= LINEAR_STD_DEV_MEGATAG_2_FACTOR;
          angularStdDev *= ANGULAR_STD_DEV_MEGATAG_2_FACTOR;
        }
        if (cameraIndex < CAMERA_STD_DEV_FACTORS.length) {
          linearStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
          angularStdDev *= CAMERA_STD_DEV_FACTORS[cameraIndex];
        }

        // Send vision observation
        consumer.accept(observation.pose().toPose2d(), observation.timestamp(), VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput("Vision/Camera" + cameraIndex + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/Camera" + cameraIndex + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface public interface VisionConsumer {
    void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
