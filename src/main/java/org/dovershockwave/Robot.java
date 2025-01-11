package org.dovershockwave;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class Robot extends LoggedRobot {
  private static final Alert LOG_FOLDER_FOUND = new Alert("Logging is enabled.",Alert.AlertType.kInfo);
  private static final Alert LOG_FOLDER_NOT_FOUND = new Alert("Logging is disabled as the log folder doesn't exist!", Alert.AlertType.kWarning);
  private static final Alert TUNING_MODE_ENABLED = new Alert("Tuning mode is enabled.", Alert.AlertType.kInfo);
  private static final Alert TUNING_MODE_ENABLED_COMP = new Alert("Tuning mode is enabled in a competition round!", Alert.AlertType.kWarning);
  private final RobotContainer container;

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.CURRENT_MODE) {
      case REAL:
        Logger.addDataReceiver(new NT4Publisher());
        //noinspection resource
        new PowerDistribution(); // Enables power distribution logging.

        final var logFolderPath = Paths.get(Constants.LOG_FOLDER_PATH);
        if (Files.notExists(logFolderPath)) {
          try {
            Files.createDirectories(logFolderPath); // Create /log folder if it's a new RIO
          } catch (IOException ignored) {}
        }

        if (Files.exists(logFolderPath)) {
          LOG_FOLDER_FOUND.set(true);
          Logger.addDataReceiver(new WPILOGWriter());
        } else {
          LOG_FOLDER_NOT_FOUND.set(true);
        }
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.registerURCL(URCL.startExternal());
    Logger.start();

    if (RobotContainer.isCompetitionMatch() && Constants.TUNING_MODE) TUNING_MODE_ENABLED_COMP.set(true);
    else TUNING_MODE_ENABLED.set(Constants.TUNING_MODE);

    container = new RobotContainer();
  }

  @Override public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    Threads.setCurrentThreadPriority(true, 99);

    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override public void autonomousInit() {
    container.autoChooser.get().schedule();
  }

  @Override public void autonomousPeriodic() {
  }

  @Override public void teleopInit() {
  }

  @Override public void teleopPeriodic() {
  }

  @Override public void teleopExit() {
  }
}
