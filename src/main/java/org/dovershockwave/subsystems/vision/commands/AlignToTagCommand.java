package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.vision.CameraType;
import org.dovershockwave.subsystems.vision.VisionConstants;
import org.dovershockwave.subsystems.vision.VisionSubsystem;
import org.dovershockwave.utils.TunablePIDF;
import org.littletonrobotics.junction.Logger;

public class AlignToTagCommand extends Command {
  private final TunablePIDF tunableOmegaPID = new TunablePIDF("AlignToTagCommand/OmegaPID", VisionConstants.ALIGNMENT_OMEGA_PID);
  private final PIDController omegaPID = new PIDController(tunableOmegaPID.getGains().p(), tunableOmegaPID.getGains().i(), tunableOmegaPID.getGains().d());
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final CameraType camera;
  private final int tagId;
  private boolean tagFound = false;

  public AlignToTagCommand(SwerveSubsystem swerve, VisionSubsystem vision, CameraType camera, int tagID) {
    this.swerve = swerve;
    this.vision = vision;
    this.camera = camera;
    this.tagId = tagID;
    addRequirements(swerve);
  }

  @Override public void initialize() {
    omegaPID.reset();
    omegaPID.setTolerance(VisionConstants.ALIGNMENT_RAD_TOLERANCE);
    omegaPID.setP(tunableOmegaPID.getGains().p());
    omegaPID.setI(tunableOmegaPID.getGains().i());
    omegaPID.setD(tunableOmegaPID.getGains().d());
  }

  @Override public void execute() {
    vision.getLatestTargetObservation(camera, tagId).ifPresentOrElse(target -> {
      this.tagFound = true;
      final var rotErrorRad = target.tx().getRadians();
      final var omega = omegaPID.calculate(rotErrorRad, 0.0);
      swerve.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));

      Logger.recordOutput("AlignToTagCommand/Target-" + tagId + "/RotErrorRad", rotErrorRad);
      Logger.recordOutput("AlignToTagCommand/Target-" + tagId + "/Omega", omega);
    }, swerve::stop);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return tagFound && omegaPID.atSetpoint();
  }
}