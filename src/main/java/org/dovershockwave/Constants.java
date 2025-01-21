package org.dovershockwave;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;
  public static final double DRIVE_DEADBAND = 0.02;
  public static final String LOG_FOLDER_PATH = "/U/logs/";
  public static final String TUNING_TABLE_NAME = "Tuning";
  public static final boolean TUNING_MODE = true;
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public enum Mode {
    /**
     * Running on a real robot.
     */
    REAL,

    /**
     * Running a physics simulator.
     */
    SIM,

    /**
     * Replaying from a log file.
     */
    REPLAY
  }
}
