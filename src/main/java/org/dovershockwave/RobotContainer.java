package org.dovershockwave;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.dovershockwave.subsystems.swerve.SwerveSubsystem;
import org.dovershockwave.subsystems.swerve.commands.FeedforwardCharacterizationCommand;
import org.dovershockwave.subsystems.swerve.commands.ResetFieldOrientatedDriveCommand;
import org.dovershockwave.subsystems.swerve.commands.SwerveDriveCommand;
import org.dovershockwave.subsystems.swerve.gyro.GyroIO;
import org.dovershockwave.subsystems.swerve.gyro.GyroIONavX;
import org.dovershockwave.subsystems.swerve.module.ModuleIO;
import org.dovershockwave.subsystems.swerve.module.ModuleIOSpark;
import org.dovershockwave.subsystems.swerve.module.ModuleType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  protected final SwerveSubsystem swerve;
//  private final VisionSubsystem vision;
  protected final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER_PORT);
  protected final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  protected final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL:
        swerve = new SwerveSubsystem(
                new GyroIONavX(),
                new ModuleIOSpark(ModuleType.FRONT_LEFT),
                new ModuleIOSpark(ModuleType.FRONT_RIGHT),
                new ModuleIOSpark(ModuleType.BACK_LEFT),
                new ModuleIOSpark(ModuleType.BACK_RIGHT));

//        vision = new VisionSubsystem(
//                swerve::addVisionMeasurement,
//                new VisionIOPhotonVision(VisionConstants.REEF_CAMERA, VisionConstants.ROBOT_TO_REEF_CAMERA),
//                new VisionIOPhotonVision(VisionConstants.HUMAN_PLAYER_STATION_CAMERA, VisionConstants.ROBOT_TO_HUMAN_PLAYER_CAMERA));
        break;
      case SIM:
        // TODO: 1/11/2025 Implement simulation modes
        swerve = new SwerveSubsystem(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
//        vision = new VisionSubsystem(
//                swerve::addVisionMeasurement,
//                new VisionIOPhotonVisionSim(VisionConstants.REEF_CAMERA, VisionConstants.ROBOT_TO_REEF_CAMERA, swerve::getPose),
//                new VisionIOPhotonVisionSim(VisionConstants.HUMAN_PLAYER_STATION_CAMERA, VisionConstants.ROBOT_TO_HUMAN_PLAYER_CAMERA, swerve::getPose));
        break;
      case REPLAY:
      default:
        swerve = new SwerveSubsystem(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
//        vision = new VisionSubsystem(swerve::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    if (!isCompetitionMatch()) {
      autoChooser.addOption("Drive Simple FF Characterization", new FeedforwardCharacterizationCommand(swerve));
      autoChooser.addOption("Drive SysId (Quasistatic Forward)", swerve.sysIdDriveQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Quasistatic Reverse)", swerve.sysIdDriveQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption("Drive SysId (Dynamic Forward)", swerve.sysIdDriveDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Dynamic Reverse)", swerve.sysIdDriveDynamic(SysIdRoutine.Direction.kReverse));

      autoChooser.addOption("Turn SysId (Quasistatic Forward)", swerve.sysIdTurnQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Turn SysId (Quasistatic Reverse)", swerve.sysIdTurnQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption("Turn SysId (Dynamic Forward)", swerve.sysIdTurnDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Turn SysId (Dynamic Reverse)", swerve.sysIdTurnDynamic(SysIdRoutine.Direction.kReverse));
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
