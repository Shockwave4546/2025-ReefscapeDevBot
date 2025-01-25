package org.dovershockwave;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.Pair;
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
import org.dovershockwave.subsystems.swerve.module.ModuleIOSim;
import org.dovershockwave.subsystems.swerve.module.ModuleIOSpark;
import org.dovershockwave.subsystems.swerve.module.ModuleType;
import org.dovershockwave.subsystems.vision.*;
import org.dovershockwave.subsystems.vision.commands.AlignToTagCommand;
import org.dovershockwave.subsystems.vision.commands.sysid.SysIdDriveDynamicCommand;
import org.dovershockwave.subsystems.vision.commands.sysid.SysIdDriveQuasistaticCommand;
import org.dovershockwave.subsystems.vision.commands.sysid.SysIdTurnQuasistaticCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  protected final SwerveSubsystem swerve;
  private final VisionSubsystem vision;
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

        vision = new VisionSubsystem(
                swerve::addVisionMeasurement,
                Pair.of(CameraType.FRONT_CAMERA, new VisionIOPhotonVision(CameraType.FRONT_CAMERA)));
        break;
      case SIM:
        swerve = new SwerveSubsystem(new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        vision = new VisionSubsystem(
                swerve::addVisionMeasurement,
                Pair.of(CameraType.FRONT_CAMERA, new VisionIOPhotonVisionSim(CameraType.FRONT_CAMERA, swerve::getPose)));
        break;
      case REPLAY:
      default:
        swerve = new SwerveSubsystem(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        vision = new VisionSubsystem(swerve::addVisionMeasurement, Pair.of(CameraType.NONE, new VisionIO() {}));
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    if (!isCompetitionMatch()) {
      autoChooser.addOption("Drive Simple FF Characterization", new FeedforwardCharacterizationCommand(swerve));
      autoChooser.addOption("Drive SysId (Quasistatic Forward)", new SysIdDriveQuasistaticCommand(swerve, SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Quasistatic Reverse)", new SysIdDriveQuasistaticCommand(swerve, SysIdRoutine.Direction.kReverse));
      autoChooser.addOption("Drive SysId (Dynamic Forward)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Drive SysId (Dynamic Reverse)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kReverse));

      autoChooser.addOption("Turn SysId (Quasistatic Forward)", new SysIdTurnQuasistaticCommand(swerve, SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Turn SysId (Quasistatic Reverse)", new SysIdTurnQuasistaticCommand(swerve, SysIdRoutine.Direction.kReverse));
      autoChooser.addOption("Turn SysId (Dynamic Forward)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kForward));
      autoChooser.addOption("Turn SysId (Dynamic Reverse)", new SysIdDriveDynamicCommand(swerve, SysIdRoutine.Direction.kReverse));
    }

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    swerve.setDefaultCommand(new SwerveDriveCommand(swerve, driverController));
    driverController.b().onTrue(new ResetFieldOrientatedDriveCommand(swerve));
    driverController.x().onTrue(new InstantCommand(swerve::stopWithX, swerve));

    driverController.y().onTrue(new AlignToTagCommand(swerve, vision, CameraType.FRONT_CAMERA));

    driverController.leftBumper().onTrue(new InstantCommand(() -> swerve.multiplyFF(-0.1)).ignoringDisable(true));
    driverController.rightBumper().onTrue(new InstantCommand(() -> swerve.multiplyFF(0.1)).ignoringDisable(true));
//    driverController.x().onTrue(new DriveLinearVelocityCommand(swerve, driverController, false));
//    driverController.y().onTrue(new DriveLinearVelocityCommand(swerve, driverController, true));

//    driverController.povDown().onTrue(new InstantCommand(() -> swerve.setDefaultCommand(new SwerveDriveCommand(swerve, driverController))));
//    driverController.povUp().onTrue(new InstantCommand(() -> CommandScheduler.getInstance().removeDefaultCommand(swerve)));
  }

  public static boolean isCompetitionMatch() {
   return DriverStation.isFMSAttached() || DriverStation.getMatchType() != DriverStation.MatchType.None;
  }
}
