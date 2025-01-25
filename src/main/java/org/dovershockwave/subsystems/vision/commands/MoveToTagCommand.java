package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.CameraType;
import org.dovershockwave.subsystems.vision.ReefScoringPosition;
import org.dovershockwave.subsystems.vision.VisionConstants;
import org.dovershockwave.subsystems.vision.VisionSubsystem;
import org.dovershockwave.utils.TunableNumber;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

// TODO: 1/25/25 Robot relative speed or whatever is messed up in this
public class MoveToTagCommand extends Command {
  private static final TunableNumber ALIGNMENT_X_METERS_TOLERANCE = new TunableNumber("MoveToTagCommand/AlignmentRadTolerance", VisionConstants.ALIGNMENT_X_METERS_TOLERANCE);
  private static final TunableNumber ALIGNMENT_Y_METERS_TOLERANCE = new TunableNumber("MoveToTagCommand/AlignmentRadTolerance", VisionConstants.ALIGNMENT_Y_METERS_TOLERANCE);
  private static final TunableNumber ALIGNMENT_RAD_TOLERANCE = new TunableNumber("MoveToTagCommand/AlignmentRadTolerance", VisionConstants.ALIGNMENT_RAD_TOLERANCE);

  private final TunablePIDF tunableXVelocityPID = new TunablePIDF("MoveToTagCommand/XVelocityPID", VisionConstants.ALIGNMENT_X_VELOCITY_PID);
  private final TunablePIDF tunableYVelocityPID = new TunablePIDF("MoveToTagCommand/YVelocityPID", VisionConstants.ALIGNMENT_Y_VELOCITY_PID);
  private final TunablePIDF tunableOmegaPID = new TunablePIDF("MoveToTagCommand/OmegaPID", VisionConstants.ALIGNMENT_OMEGA_PID);

  private final PIDController xVelocityPID = new PIDController(0, 0, 0);
  private final PIDController yVelocityPID = new PIDController(0, 0, 0);
  private final PIDController omegaPID = new PIDController(2.91, 0.0, 0.094);

  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final CameraType camera;
  private boolean tagFound = false;

  public MoveToTagCommand(SwerveSubsystem swerve, VisionSubsystem vision, CameraType camera) {
    this.swerve = swerve;
    this.vision = vision;
    this.camera = camera;
    addRequirements(swerve);
  }

  @Override public void initialize() {
    omegaPID.setTolerance(ALIGNMENT_RAD_TOLERANCE.get());
    omegaPID.enableContinuousInput(-Math.PI, Math.PI);
    omegaPID.setP(tunableOmegaPID.getGains().p());
    omegaPID.setI(tunableOmegaPID.getGains().i());
    omegaPID.setD(tunableOmegaPID.getGains().d());
    omegaPID.reset();

    xVelocityPID.setTolerance(ALIGNMENT_X_METERS_TOLERANCE.get());
    xVelocityPID.setP(tunableXVelocityPID.getGains().p());
    xVelocityPID.setI(tunableXVelocityPID.getGains().i());
    xVelocityPID.setD(tunableXVelocityPID.getGains().d());
    xVelocityPID.reset();

    yVelocityPID.setTolerance(ALIGNMENT_Y_METERS_TOLERANCE.get());
    yVelocityPID.setP(tunableYVelocityPID.getGains().p());
    yVelocityPID.setI(tunableYVelocityPID.getGains().i());
    yVelocityPID.setD(tunableYVelocityPID.getGains().d());
    yVelocityPID.reset();
  }

  @Override public void execute() {
    final var bestTarget = vision.getBestTargetObservation(camera);

    ReefScoringPosition.getPositionFor(bestTarget.tagId(), ReefScoringPosition.ReefScoringSide.LEFT, ReefScoringPosition.ReefLevel.L1).ifPresentOrElse(position -> {
      this.tagFound = true;
      final var xVelocity = xVelocityPID.calculate(bestTarget.translation().getX(), 0.5);
      final var yVelocity = yVelocityPID.calculate(bestTarget.translation().getY(), 0.0);
      final var omega = omegaPID.calculate(swerve.getRotation().getRadians(), position.robotHeading.getRadians());

      swerve.runVelocity(new ChassisSpeeds(xVelocity, yVelocity, omega), false);

      Logger.recordOutput("MoveToTagCommand/Target-" + position.id + "/ThetaXCurrent", bestTarget.translation().getX());
      Logger.recordOutput("MoveToTagCommand/Target-" + position.id  + "/ThetaXGoal", 0.0);
      Logger.recordOutput("MoveToTagCommand/Target-" + position.id  + "/ThetaXError", xVelocityPID.getError());
      Logger.recordOutput("MoveToTagCommand/Target-" + position.id  + "/XVelocity", xVelocity);

      Logger.recordOutput("MoveToTagCommand/Target-" + position.id + "/ThetaYCurrent", bestTarget.translation().getY());
      Logger.recordOutput("MoveToTagCommand/Target-" + position.id  + "/ThetaYGoal", 0.0);
      Logger.recordOutput("MoveToTagCommand/Target-" + position.id  + "/ThetaYError", yVelocityPID.getError());
      Logger.recordOutput("MoveToTagCommand/Target-" + position.id  + "/YVelocity", yVelocity);

      Logger.recordOutput("MoveToTagCommand/Target-" + position.id + "/RotRadCurrent", swerve.getRotation().getRadians());
      Logger.recordOutput("MoveToTagCommand/Target-" + position.id  + "/RotRadGoal", position.robotHeading.getRadians());
      Logger.recordOutput("MoveToTagCommand/Target-" + position.id  + "/RotRadError", omegaPID.getError());
      Logger.recordOutput("MoveToTagCommand/Target-" + position.id  + "/Omega", omega);
    }, swerve::stop);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return tagFound && omegaPID.atSetpoint() && xVelocityPID.atSetpoint() && yVelocityPID.atSetpoint();
  }
}