package org.dovershockwave.subsystems.vision.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;

// TODO: 1/15/25  
public class AlignToTagCommand extends Command {
  public static final double ALIGNMENT_TOLERANCE = 0.1;
  public static final PIDController omegaPID = new PIDController(2.91, 0.0, 0.094);
  private final SwerveSubsystem swerve;

  public AlignToTagCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }
}