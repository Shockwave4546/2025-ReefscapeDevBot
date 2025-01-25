package org.dovershockwave.subsystems.vision.commands.sysid;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;

public class SysIdTurnQuasistaticCommand extends SequentialCommandGroup {
  public SysIdTurnQuasistaticCommand(SwerveSubsystem swerve, SysIdRoutine.Direction direction) {
    addCommands(
            new InstantCommand(() -> swerve.runDriveCharacterization(0.0)),
            new WaitCommand(1.0),
            new InstantCommand(() -> swerve.turnSysId.quasistatic(direction))

    );

    addRequirements(swerve);
  }
}
