package org.dovershockwave;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
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
