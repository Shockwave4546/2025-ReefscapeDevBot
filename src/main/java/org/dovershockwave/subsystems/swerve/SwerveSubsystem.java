package org.dovershockwave.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.dovershockwave.Constants;
import org.dovershockwave.subsystems.swerve.gyro.GyroIO;
import org.dovershockwave.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import org.dovershockwave.subsystems.swerve.module.Module;
import org.dovershockwave.utils.LocalADStarAK;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.units.Units.Volts;
import static org.dovershockwave.subsystems.swerve.SwerveConstants.*;

public class SwerveSubsystem extends SubsystemBase {
  public static final Lock ODOMETRY_LOCK = new ReentrantLock();
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro", Alert.AlertType.kError);
  private final GyroIO gyroIO;
  private final SwerveIO swerveIO;
  private final SysIdRoutine sysId;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATIONS);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
  };
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public SwerveSubsystem(GyroIO gyroIO, SwerveIO swerveIO) {
    this.gyroIO = gyroIO;
    this.swerveIO = swerveIO;

    SparkOdometryThread.getInstance().start();

    AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::runVelocity,
            new PPHolonomicDriveController(TRANSLATION_PID, ROTATION_PID),
            PATH_PLANNER_ROBOT_CONFIG,
            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
            this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback((activePath) -> Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    sysId = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                    (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override public void periodic() {
    ODOMETRY_LOCK.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (Module module : swerveIO.getModules()) {
      module.periodic();
    }
    ODOMETRY_LOCK.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (Module module : swerveIO.getModules()) {
        module.stop();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    final double[] sampleTimestamps = swerveIO.getModules()[0].getOdometryTimestamps(); // All signals are sampled together
    for (int i = 0; i < sampleTimestamps.length; i++) {
      // Read wheel positions and deltas from each module
      var modulePositions = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = swerveIO.getModules()[moduleIndex].getOdometryPositions()[i];
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) rawGyroRotation = gyroInputs.odometryYawPositions[i];

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.CURRENT_MODE != Constants.Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    var discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    var setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      swerveIO.getModules()[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      swerveIO.getModules()[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    final var headings = Arrays.stream(SwerveConstants.MODULE_TRANSLATIONS).map(Translation2d::getAngle).toArray(Rotation2d[]::new);
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(swerveIO.getModules()).map(Module::getState).toArray(SwerveModuleState[]::new);
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(swerveIO.getModules()).map(Module::getPosition).toArray(SwerveModulePosition[]::new);
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    return Arrays.stream(swerveIO.getModules()).mapToDouble(Module::getWheelRadiusCharacterizationPosition).toArray();
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return Arrays.stream(swerveIO.getModules()).mapToDouble(Module::getFFCharacterizationVelocity).average().orElse(0.0);
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }
}
