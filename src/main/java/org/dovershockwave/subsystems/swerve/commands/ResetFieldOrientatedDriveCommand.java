package org.dovershockwave.subsystems.swerve.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;

public class ResetFieldOrientatedDriveCommand extends InstantCommand {
  public ResetFieldOrientatedDriveCommand(SwerveSubsystem swerve) {
    super(
            () -> swerve.setPose(new Pose2d(swerve.getPose().getTranslation(), new Rotation2d())),
            swerve
    );
  }

  @Override public boolean runsWhenDisabled() {
    return true;
  }
}
