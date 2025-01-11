package org.dovershockwave.subsystems.swerve;

import org.dovershockwave.subsystems.swerve.module.Module;

public interface SwerveIO {

  /**
   * Supplied in the same order as the kinematics: FL, FR< BL, BR
   */
  default Module[] getModules() { return new Module[] {}; }
}