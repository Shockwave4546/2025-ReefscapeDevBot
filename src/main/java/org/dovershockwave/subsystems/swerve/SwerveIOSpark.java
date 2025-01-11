package org.dovershockwave.subsystems.swerve;

import org.dovershockwave.subsystems.swerve.module.Module;
import org.dovershockwave.subsystems.swerve.module.ModuleIOSpark;
import org.dovershockwave.subsystems.swerve.module.ModuleType;

public class SwerveIOSpark implements SwerveIO {
  private final Module frontLeft = new Module(new ModuleIOSpark(ModuleType.FRONT_LEFT), ModuleType.FRONT_LEFT);
  private final Module frontRight = new Module(new ModuleIOSpark(ModuleType.FRONT_RIGHT), ModuleType.FRONT_RIGHT);
  private final Module backLeft = new Module(new ModuleIOSpark(ModuleType.BACK_LEFT), ModuleType.BACK_LEFT);
  private final Module backRight = new Module(new ModuleIOSpark(ModuleType.BACK_RIGHT), ModuleType.BACK_RIGHT);

  @Override public Module[] getModules() {
    return new Module[] {frontLeft, frontRight, backLeft, backRight};
  }
}
