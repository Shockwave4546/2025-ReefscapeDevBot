package org.dovershockwave;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.dovershockwave.subsystems.swerve.SwerveIO;
import org.dovershockwave.subsystems.swerve.SwerveIOSpark;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.swerve.commands.FeedforwardCharacterizationCommand;
import org.dovershockwave.subsystems.swerve.commands.ResetFieldOrientatedDriveCommand;
import org.dovershockwave.subsystems.swerve.commands.SwerveDriveCommand;
import org.dovershockwave.subsystems.swerve.gyro.GyroIO;
import org.dovershockwave.subsystems.swerve.gyro.GyroIONavX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  protected final SwerveSubsystem swerve;
  protected final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  protected final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  protected final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        swerve = new SwerveSubsystem(new GyroIONavX(), new SwerveIOSpark());
        break;
      case SIM:
        // TODO: 1/11/2025 Implement simulation modes
        swerve = new SwerveSubsystem(new GyroIO() {}, new SwerveIO() {});
        break;
      case REPLAY:
      default:
        swerve = new SwerveSubsystem(new GyroIO() {}, new SwerveIO() {});
    }

    if (!isCompetitionMatch()) {
      autoChooser.addOption("Drive Simple FF Characterization", new FeedforwardCharacterizationCommand(swerve));
      autoChooser.addOption("Drive SysId (Quasistatic Forward)", swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Quasistatic Reverse)", swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption("Drive SysId (Dynamic Forward)", swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Dynamic Reverse)", swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, driverController));
    driverController.b().onTrue(new ResetFieldOrientatedDriveCommand(swerve));
    driverController.x().onTrue(new InstantCommand(swerve::stopWithX, swerve));
  }

  public static boolean isCompetitionMatch() {
   return DriverStation.isFMSAttached() || DriverStation.getMatchType() != DriverStation.MatchType.None;
  }
}
