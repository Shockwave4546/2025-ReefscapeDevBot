package org.dovershockwave.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import org.dovershockwave.subsystems.swerve.SwerveConstants;
import org.dovershockwave.utils.PIDFGains;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final ModuleType type;

  private final TunablePIDF drivePIDF;
  private final TunablePIDF turnPIDF;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(ModuleIO io, ModuleType type) {
    this.io = io;
    this.type = type;
    drivePIDF = new TunablePIDF("Drive/" + type.name + "Module/DrivePID/", SwerveConstants.DRIVE_PIDF);
    turnPIDF = new TunablePIDF("Drive/" + type.name + "Module/TurnPID/", SwerveConstants.TURN_PIDF);

    driveDisconnectedAlert = new Alert("Disconnected drive motor on " + type.name + " module.", Alert.AlertType.kError);
    turnDisconnectedAlert = new Alert("Disconnected turn motor on " + type.name + " module.", Alert.AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/" + type.name + "Module", inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * SwerveConstants.WHEEL_RADIUS_METERS;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    drivePIDF.periodic(io::setDrivePIDF, io::setDriveVelocity);
    turnPIDF.periodic(io::setTurnPIDF, value -> io.setTurnPosition(new Rotation2d(value)));

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(getAngle());

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / SwerveConstants.WHEEL_RADIUS_METERS);
    io.setTurnPosition(state.angle);
  }

  public void setDrivePIDF(PIDFGains gains) {
    io.setDrivePIDF(gains);
    drivePIDF.setGains(gains);
  }

  public PIDFGains getDrivePIDF() {
    return drivePIDF.getGains();
  }

  public void setTurnPIDF(PIDFGains gains) {
    io.setTurnPIDF(gains);
    turnPIDF.setGains(gains);
  }

  public PIDFGains getTurnPIDF() {
    return turnPIDF.getGains();
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runDriveCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }

  public void runTurnCharacterization(double output) {
    io.setDriveOpenLoop(output);
  }

  public void setTurnPosition(Rotation2d rotation) {
    io.setTurnPosition(rotation);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * SwerveConstants.WHEEL_RADIUS_METERS;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * SwerveConstants.WHEEL_RADIUS_METERS;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
