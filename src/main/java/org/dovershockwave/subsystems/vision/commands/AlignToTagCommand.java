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

public class AlignToTagCommand extends Command {
  private static final TunableNumber ALIGNMENT_RAD_TOLERANCE = new TunableNumber("AlignToTagCommand/AlignmentRadTolerance", VisionConstants.ALIGNMENT_RAD_TOLERANCE);
  private final TunablePIDF tunableOmegaPID = new TunablePIDF("AlignToTagCommand/OmegaPID", VisionConstants.ALIGNMENT_OMEGA_PID);
  private final PIDController omegaPID = new PIDController(tunableOmegaPID.getGains().p(), tunableOmegaPID.getGains().i(), tunableOmegaPID.getGains().d());
  private final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
  private final CameraType camera;
  private boolean tagFound = false;

  public AlignToTagCommand(SwerveSubsystem swerve, VisionSubsystem vision, CameraType camera) {
    this.swerve = swerve;
    this.vision = vision;
    this.camera = camera;
    addRequirements(swerve);

    omegaPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override public void initialize() {
    omegaPID.reset();
    omegaPID.setTolerance(ALIGNMENT_RAD_TOLERANCE.get());
    omegaPID.setP(tunableOmegaPID.getGains().p());
    omegaPID.setI(tunableOmegaPID.getGains().i());
    omegaPID.setD(tunableOmegaPID.getGains().d());
  }

  @Override public void execute() {
    final var bestTarget = vision.getBestTargetObservation(camera);

    ReefScoringPosition.getPositionFor(bestTarget.tagId(), ReefScoringPosition.ReefScoringSide.LEFT, ReefScoringPosition.ReefLevel.L1).ifPresentOrElse(position -> {
      this.tagFound = true;
      final var omega = omegaPID.calculate(swerve.getRotation().getRadians(), position.robotHeading.getRadians());
      swerve.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));

      Logger.recordOutput("AlignToTagCommand/Target-" + position.id + "/RotRadCurrent", swerve.getRotation().getRadians());
      Logger.recordOutput("AlignToTagCommand/Target-" + position.id  + "/RotRadGoal", position.robotHeading.getRadians());
      Logger.recordOutput("AlignToTagCommand/Target-" + position.id  + "/RotRadError", omegaPID.getError());
      Logger.recordOutput("AlignToTagCommand/Target-" + position.id  + "/Omega", omega);
    }, swerve::stop);
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return tagFound && omegaPID.atSetpoint();
  }
}