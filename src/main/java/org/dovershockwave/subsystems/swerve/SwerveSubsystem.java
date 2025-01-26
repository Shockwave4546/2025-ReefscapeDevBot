package org.dovershockwave.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.dovershockwave.Constants;
import org.dovershockwave.subsystems.swerve.gyro.GyroIO;
import org.dovershockwave.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import org.dovershockwave.subsystems.swerve.lidar.LidarIO;
import org.dovershockwave.subsystems.swerve.lidar.LidarIOLaserCan;
import org.dovershockwave.subsystems.swerve.module.Module;
import org.dovershockwave.subsystems.swerve.module.ModuleIO;
import org.dovershockwave.subsystems.swerve.module.ModuleType;
import org.dovershockwave.utils.PIDFGains;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import static edu.wpi.first.units.Units.Volts;

public class SwerveSubsystem extends SubsystemBase {
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro", Alert.AlertType.kError);
  private final LidarIO lidarIO;
  private final GyroIO gyroIO;

  private final Module[] modules;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.MODULE_TRANSLATIONS);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
  };
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public final SysIdRoutine driveSysId = new SysIdRoutine(
          new SysIdRoutine.Config(null, null, null,
                  (state) -> Logger.recordOutput("Drive/DriveSysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
                  (voltage) -> runDriveCharacterization(voltage.in(Volts)), null, this));

  public final SysIdRoutine turnSysId = new SysIdRoutine(
          new SysIdRoutine.Config(null, null, null,
                  (state) -> Logger.recordOutput("Drive/TurnSysIdState", state.toString())),
          new SysIdRoutine.Mechanism(
                  (voltage) -> runTurnCharacterization(voltage.in(Volts)), null, this));

  private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(SwerveConstants.PATH_PLANNER_ROBOT_CONFIG, SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC);
  private SwerveSetpoint previousSetpoint;

  public SwerveSubsystem(GyroIO gyroIO, ModuleIO frontLeft, ModuleIO frontRight, ModuleIO backLeft, ModuleIO backRight) {
    this.lidarIO = new LidarIOLaserCan();
    this.gyroIO = gyroIO;
    this.modules = new Module[]{
            new Module(frontLeft, ModuleType.FRONT_LEFT),
            new Module(frontRight, ModuleType.FRONT_RIGHT),
            new Module(backLeft, ModuleType.BACK_LEFT),
            new Module(backRight, ModuleType.BACK_RIGHT)
    };

    AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            speeds -> runVelocity(speeds, true),
            new PPHolonomicDriveController(SwerveConstants.TRANSLATION_PID, SwerveConstants.ROTATION_PID, 0.02),
            SwerveConstants.PATH_PLANNER_ROBOT_CONFIG,
            () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red,
            this
    );
    PathPlannerLogging.setLogActivePathCallback((activePath) -> Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0])));
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
    SparkOdometryThread.getInstance().start();
  }

  @Override public void periodic() {
    SparkOdometryThread.ODOMETRY_LOCK.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (Module module : modules) {
      module.periodic();
    }
    SparkOdometryThread.ODOMETRY_LOCK.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (Module module : modules) {
        module.stop();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    final double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    for (int i = 0; i < sampleTimestamps.length; i++) {
      // Read wheel positions and deltas from each module
      var modulePositions = new SwerveModulePosition[4];
      var moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters, modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        final var twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.CURRENT_MODE != Constants.Mode.SIM);
  }

  public void multiplyFF(double multiplier) {
    for (Module module : modules) {
      final var curr = module.getDrivePIDF();
      final var newFF = (1 + multiplier) * curr.ff();
      module.setDrivePIDF(new PIDFGains(curr.p(), curr.i(), curr.d(), newFF));
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds, boolean useSwerveSetpointGenerator) {
    if (useSwerveSetpointGenerator) {
      var setpointStates = kinematics.toSwerveModuleStates(speeds);
      Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

      previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);
      setpointStates = previousSetpoint.moduleStates();

      for (int i = 0; i < 4; i++) {
        modules[i].runSetpoint(setpointStates[i]);
      }

      Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    } else {
      ChassisSpeeds.discretize(speeds, 0.02);
      var setpointStates = kinematics.toSwerveModuleStates(speeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

      Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

      for (int i = 0; i < 4; i++) {
        modules[i].runSetpoint(setpointStates[i]);
      }

      Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }
  }

  public void runDriveCharacterization(double volts) {
    for (Module module : modules) {
      module.runDriveCharacterization(volts);
    }
  }

  public void runTurnCharacterization(double volts) {
    for (Module module : modules) {
      module.runTurnCharacterization(volts);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds(), false);
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

  /** Returns the module states (turn angles and drive velocities) for all the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules).map(Module::getState).toArray(SwerveModuleState[]::new);
  }

  /** Returns the module positions (turn angles and drive positions) for all the modules. */
  private SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(modules).map(Module::getPosition).toArray(SwerveModulePosition[]::new);
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double getFFCharacterizationVelocity() {
    return Arrays.stream(modules).mapToDouble(Module::getFFCharacterizationVelocity).average().orElse(0.0);
  }

  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }
}
