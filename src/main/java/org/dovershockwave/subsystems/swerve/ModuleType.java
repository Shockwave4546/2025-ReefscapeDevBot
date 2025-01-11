package org.dovershockwave.subsystems.swerve;

import static org.dovershockwave.subsystems.swerve.SwerveConstants.*;

public enum ModuleType {
  FRONT_LEFT(FRONT_LEFT_DRIVE_ID, FRONT_LEFT_TURN_ID),
  FRONT_RIGHT(FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_TURN_ID),
  BACK_LEFT(BACK_LEFT_DRIVE_ID, BACK_LEFT_TURN_ID),
  BACK_RIGHT(BACK_RIGHT_DRIVE_ID, BACK_RIGHT_TURN_ID);

  public final int driveID;
  public final int turnID;

  ModuleType(int driveID, int turnID) {
    this.driveID = driveID;
    this.turnID = turnID;
  }
}
