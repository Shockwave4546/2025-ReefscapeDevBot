package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.CameraType;
import org.dovershockwave.subsystems.vision.VisionConstants;
import org.dovershockwave.subsystems.vision.VisionSubsystem;

// TODO: 1/20/2025 Add TunablePIDF to this whole class
public class MoveToTagCommand extends Command {
  private final PIDController xVelocityPID = new PIDController(0, 0, 0);
  private final PIDController yVelocityPID = new PIDController(0, 0, 0);
  private final PIDController omegaPID = new PIDController(2.91, 0.0, 0.094);
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final CameraType camera;
  private final int tagId;
  private final Translation2d offsetMeters;
  private boolean tagFound = false;

  public MoveToTagCommand(SwerveSubsystem swerve, VisionSubsystem vision, CameraType camera, int tagID, Translation2d offsetMeters) {
    this.swerve = swerve;
    this.vision = vision;
    this.camera = camera;
    this.tagId = tagID;
    this.offsetMeters = offsetMeters;
    addRequirements(swerve);
  }

  @Override public void initialize() {
    omegaPID.reset();
    omegaPID.setTolerance(VisionConstants.ALIGNMENT_RAD_TOLERANCE);

    xVelocityPID.reset();
    xVelocityPID.setTolerance(VisionConstants.ALIGNMENT_X_METERS_TOLERANCE);

    yVelocityPID.reset();
    yVelocityPID.setTolerance(VisionConstants.ALIGNMENT_Y_METERS_TOLERANCE);
  }

  @Override public void execute() {
    vision.getLatestTargetObservation(camera, tagId).ifPresentOrElse(target -> {
      this.tagFound = true;
      final var rotError = target.tx().getRadians();
      final var xError = target.translation().getX() - offsetMeters.getX();
      final var yError = target.translation().getY() - offsetMeters.getY();

      final var omega = omegaPID.calculate(rotError, 0.0);
      final var xVelocity = xVelocityPID.calculate(xError, 0.0);
      final var yVelocity = yVelocityPID.calculate(yError, 0.0);

      swerve.runVelocity(new ChassisSpeeds(xVelocity, yVelocity, omega));
    }, swerve::stop);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return tagFound && omegaPID.atSetpoint() && xVelocityPID.atSetpoint() && yVelocityPID.atSetpoint();
  }
}