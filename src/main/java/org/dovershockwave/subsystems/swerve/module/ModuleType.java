package org.dovershockwave.subsystems.swerve.module;

import static org.dovershockwave.subsystems.swerve.SwerveConstants.*;

public enum ModuleType {
  FRONT_LEFT("FL", FRONT_LEFT_DRIVE_ID, FRONT_LEFT_TURN_ID, -Math.PI),
  FRONT_RIGHT("FR", FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_TURN_ID, -Math.PI / 2),
  BACK_LEFT("BL", BACK_LEFT_DRIVE_ID, BACK_LEFT_TURN_ID, Math.PI / 2),
  BACK_RIGHT("BR", BACK_RIGHT_DRIVE_ID, BACK_RIGHT_TURN_ID, 0.0);

  public final String name;
  public final int driveID;
  public final int turnID;
  public final double angleOffset;

  ModuleType(String name, int driveID, int turnID, double angleOffset) {
    this.name = name;
    this.driveID = driveID;
    this.turnID = turnID;
    this.angleOffset = angleOffset;
  }
}