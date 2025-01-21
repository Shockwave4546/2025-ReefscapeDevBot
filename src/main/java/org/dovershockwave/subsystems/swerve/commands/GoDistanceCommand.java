package org.dovershockwave.subsystems.swerve.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.utils.TunableNumber;

public class GoDistanceCommand extends Command {
  public static final TunableNumber distance = new TunableNumber("GoDistanceCommand/Distance", 3.5);
  public static final TunableNumber velocity = new TunableNumber("GoDistanceCommand/XVelocity", 2.0);

  private final SwerveSubsystem swerve;
  private final CommandXboxController controller;
  private final boolean invert;

  public GoDistanceCommand(SwerveSubsystem swerve, CommandXboxController controller, boolean invert) {
    this.swerve = swerve;
    this.controller = controller;
    this.invert = invert;
  }

  @Override public void initialize() {
    swerve.setPose(new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  @Override public void execute() {
    swerve.runVelocity(new ChassisSpeeds(invert ? -1 * velocity.get() : velocity.get(), 0.0, 0.0));
  }

  @Override public void end(boolean interrupted) {
    swerve.stop();
  }

  @Override public boolean isFinished() {
    return Math.abs(swerve.getPose().getX()) >= distance.get() || controller.rightTrigger().getAsBoolean();
  }
}