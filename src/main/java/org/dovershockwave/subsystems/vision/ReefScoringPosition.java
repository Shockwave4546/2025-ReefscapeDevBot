package org.dovershockwave.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class ReefScoringPosition {
  /**
   * Reef scoring positions with blue alliance wall as reference and robot facing the field.
   *
   * TODO: Lowk this might not work based on alliance color, but we'll see (at least the rotation part)
   */
  private static final Map<Integer, Pose2d> REEF_SCORING_POSE_2D = new HashMap<>();
  static {
    REEF_SCORING_POSE_2D.put(18, new Pose2d(3.6576, 4.0259, Rotation2d.fromRadians(0.0)));
    REEF_SCORING_POSE_2D.put(17, new Pose2d(4.073905999999999, 4.745482, Rotation2d.fromRadians(Math.PI / 3)));
    REEF_SCORING_POSE_2D.put(22, new Pose2d(4.904739999999999, 3.3063179999999996, Rotation2d.fromRadians(2 * Math.PI / 3)));
    REEF_SCORING_POSE_2D.put(21, new Pose2d(5.321046, 4.0259, Rotation2d.fromRadians(Math.PI)));
    REEF_SCORING_POSE_2D.put(20, new Pose2d(4.904739999999999, 4.745482, Rotation2d.fromRadians(-2 * Math.PI / 3)));
    REEF_SCORING_POSE_2D.put(19, new Pose2d(4.073905999999999, 4.745482, Rotation2d.fromRadians(-Math.PI / 3)));

    // TODO: 1/24/2025 do this for red alliance cus im lazy rn
    // https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape.json
    /**
     *             scoringPositions[0] = new ReefScoringPosition(7, Math.PI);
     *             scoringPositions[1] = new ReefScoringPosition(8, -2 * Math.PI / 3);
     *             scoringPositions[2] = new ReefScoringPosition(9, -Math.PI / 3);
     *             scoringPositions[3] = new ReefScoringPosition(10, 0);
     *             scoringPositions[4] = new ReefScoringPosition(11, Math.PI / 3);
     *             scoringPositions[5] = new ReefScoringPosition(6, 2 * Math.PI / 3);
     */
    REEF_SCORING_POSE_2D.put(7, new Pose2d(3.6576, 8.342592, Rotation2d.fromRadians(0.0)));
    REEF_SCORING_POSE_2D.put(8, new Pose2d(4.073905999999999, 9.062214, Rotation2d.fromRadians(0.0)));
    REEF_SCORING_POSE_2D.put(9, new Pose2d(4.073905999999999, 9.781836, Rotation2d.fromRadians(0.0)));
    REEF_SCORING_POSE_2D.put(10, new Pose2d(3.6576, 10.501458, Rotation2d.fromRadians(0.0)));
    REEF_SCORING_POSE_2D.put(11, new Pose2d(4.073905999999999, 11.22108, Rotation2d.fromRadians(0.0)));
    REEF_SCORING_POSE_2D.put(6, new Pose2d(4.073905999999999, 11.940702, Rotation2d.fromRadians(0.0)));
  }

  public final int id;
  public final Translation3d position;
  public final Rotation2d robotHeading;

  public ReefScoringPosition(int id, Translation3d position, Rotation2d robotHeading) {
    this.id = id;
    this.position = position;
    this.robotHeading = robotHeading;
  }
  
  public static Optional<ReefScoringPosition> getPositionFor(int id, ReefScoringSide side, ReefLevel level) {
    if (!REEF_SCORING_POSE_2D.containsKey(id)) return Optional.empty();

    final var offsetPose2d = REEF_SCORING_POSE_2D.get(id).transformBy(new Transform2d(0.0, side.yOffset, new Rotation2d()));
    return Optional.of(new ReefScoringPosition(
            id,
            new Translation3d(offsetPose2d.getX(), offsetPose2d.getY(), level.height),
            offsetPose2d.getRotation()
    ));
  }
  
  public enum ReefScoringSide {
    LEFT(Units.inchesToMeters(-6.469)),
    RIGHT(Units.inchesToMeters(6.469));

    public final double yOffset;

    ReefScoringSide(double yOffset) {
      this.yOffset = yOffset;
    }
  }

  public enum ReefLevel {
    L4(Units.inchesToMeters(72), Units.degreesToRadians(-90)),
    L3(Units.inchesToMeters(47.625), Units.degreesToRadians(-35)),
    L2(Units.inchesToMeters(31.875), Units.degreesToRadians(-35)),
    L1(Units.inchesToMeters(18), Units.degreesToRadians(0));

    public final double height;
    public final double pitch;

    ReefLevel(double height, double pitch) {
      this.height = height;
      this.pitch = pitch;
    }
  }
}