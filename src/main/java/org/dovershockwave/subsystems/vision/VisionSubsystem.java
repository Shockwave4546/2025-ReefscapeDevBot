package org.dovershockwave.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.dovershockwave.subsystems.vision.VisionIO.TargetObservation;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.LinkedList;
import java.util.Optional;

import static org.dovershockwave.subsystems.vision.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {
  private final VisionConsumer consumer;
  private final CameraType[] cameras;
  private final EnumMap<CameraType, VisionIO> io = new EnumMap<>(CameraType.class);
  private final EnumMap<CameraType, VisionIOInputsAutoLogged> inputs = new EnumMap<>(CameraType.class);
  private final EnumMap<CameraType, Alert> disconnectedAlerts = new EnumMap<>(CameraType.class);

  @SafeVarargs public VisionSubsystem(VisionConsumer consumer, Pair<CameraType, VisionIO>... ios) {
    this.consumer = consumer;
    this.cameras = Arrays.stream(ios).map(Pair::getFirst).toArray(CameraType[]::new);
    for (var io : ios) {
      this.io.put(io.getFirst(), io.getSecond());
      this.inputs.put(io.getFirst(), new VisionIOInputsAutoLogged());
      this.disconnectedAlerts.put(io.getFirst(), new Alert("Vision camera " + io.getFirst().name + " is disconnected.", AlertType.kWarning));
    }
  }

  public TargetObservation[] getLatestTargetObservations(CameraType camera) {
    return inputs.get(camera).latestTargetObservations;
  }

  public Optional<TargetObservation> getLatestTargetObservation(CameraType camera, int id) {
    return Arrays.stream(inputs.get(camera).latestTargetObservations)
            .filter(observation -> observation.tagId() == id)
            .findFirst();
  }

  public TargetObservation getBestTargetObservation(CameraType camera) {
    return inputs.get(camera).bestTargetObservation;
  }

  public Optional<TargetObservation> getBestTargetObservation(CameraType camera, int id) {
    final var observation = inputs.get(camera).bestTargetObservation;
    return observation.tagId() == id ? Optional.of(observation) : Optional.empty();
  }

  @Override public void periodic() {
    for (final var camera : cameras) {
      io.get(camera).updateInputs(inputs.get(camera));
      Logger.processInputs("Vision/" + camera.name, inputs.get(camera));
    }
    // Initialize logging values
    final var allTagPoses = new LinkedList<Pose3d>();
    final var allRobotPoses = new LinkedList<Pose3d>();
    final var allRobotPosesAccepted = new LinkedList<Pose3d>();
    final var allRobotPosesRejected = new LinkedList<Pose3d>();

    for (final var camera : cameras) {
      disconnectedAlerts.get(camera).set(!inputs.get(camera).connected);

      // Initialize logging values
      final var tagPoses = new LinkedList<Pose3d>();
      final var robotPoses = new LinkedList<Pose3d>();
      final var robotPosesAccepted = new LinkedList<Pose3d>();
      final var robotPosesRejected = new LinkedList<Pose3d>();

      // Add tag poses
      for (int tagId : inputs.get(camera).tagIds) {
        var tagPose = APRIL_TAG_FIELD.getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over pose observations
      for (var observation : inputs.get(camera).poseObservations) {
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
        final double linearStdDev = LINEAR_STD_DEV_BASELINE * stdDevFactor * camera.stdDevFactor;
        final double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor * camera.stdDevFactor;

        consumer.accept(observation.pose().toPose2d(), observation.timestamp(), VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera data
      Logger.recordOutput("Vision/" + camera.name + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/" + camera.name + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/" + camera.name + "/RobotPosesAccepted", robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput("Vision/" + camera.name + "/RobotPosesRejected", robotPosesRejected.toArray(new Pose3d[0]));
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