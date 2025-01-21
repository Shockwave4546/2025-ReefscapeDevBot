package org.dovershockwave.subsystems.swerve.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.utils.TunableNumber;

/**
 * Command for tuning PID and FF constants for the drive motors of the swerve drive.
 * Right trigger to end the command.
 */
public class DriveLinearVelocityCommand extends Command {
  public static final TunableNumber distance = new TunableNumber("DriveLinearVelocityCommand/DistanceMeters", 3.5);
  public static final TunableNumber velocity = new TunableNumber("DriveLinearVelocityCommand/XVelocityMPS", 2.0);

  private final SwerveSubsystem swerve;
  private final CommandXboxController controller;
  private final boolean invertDriveDirection;

  public DriveLinearVelocityCommand(SwerveSubsystem swerve, CommandXboxController controller, boolean invertDriveDirection) {
    this.swerve = swerve;
    this.controller = controller;
    this.invertDriveDirection = invertDriveDirection;
  }

  @Override public void initialize() {
    swerve.setPose(new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  @Override public void execute() {
    swerve.runVelocity(new ChassisSpeeds(invertDriveDirection ? -1 * velocity.get() : velocity.get(), 0.0, 0.0));
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return Math.abs(swerve.getPose().getX()) >= distance.get() || controller.rightTrigger().getAsBoolean();
  }
}